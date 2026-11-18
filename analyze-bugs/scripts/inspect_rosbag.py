#!/usr/bin/env python3
"""Read-only SQLite rosbag2 inventory; standard library only, no ROS traffic."""

import argparse
import heapq
import json
import math
import sqlite3
import sys
from collections import defaultdict
from pathlib import Path


def percentile(values, fraction):
    """Linear interpolation on the sorted sample, like numpy's default."""
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def inspect(path, timing=False, selected=()):
    root = Path(path).expanduser().resolve()
    if root.is_file() and root.suffix == ".db3":
        files = [root]
    elif root.is_dir():
        files = sorted(root.glob("*.db3"))
        if not files:
            raise ValueError("目录中没有 .db3 分片；MCAP/压缩包请使用匹配的 rosbag2 读取后端")
        if list(root.glob("*.mcap")) or list(root.glob("*.zstd")):
            raise ValueError("目录混有其他存储/压缩格式，不能只分析 db3 后宣称覆盖全部录制")
    else:
        raise ValueError("输入应为已有 rosbag2 SQLite 目录或单个 .db3 文件")
    sources = []
    topics = defaultdict(list)
    connections = []
    try:
        for shard in files:
            connection = sqlite3.connect(shard.as_uri() + "?mode=ro", uri=True)
            connection.execute("PRAGMA query_only=ON")
            connections.append(connection)
            count, first, last = connection.execute(
                "SELECT COUNT(*), MIN(timestamp), MAX(timestamp) FROM messages"
            ).fetchone()
            sources.append({
                "path": str(shard),
                "size_bytes": shard.stat().st_size,
                "mtime_ns": shard.stat().st_mtime_ns,
                "messages": count,
                "first_receive_ns": first,
                "last_receive_ns": last,
            })
            for topic_id, name, type_name, encoding in connection.execute(
                "SELECT id, name, type, serialization_format FROM topics"
            ):
                topics[(name, type_name, encoding)].append((connection, topic_id))

        missing = set(selected) - {key[0] for key in topics}
        if missing:
            raise ValueError("未录制指定话题: " + ", ".join(sorted(missing)))
        firsts = [source["first_receive_ns"] for source in sources
                  if source["first_receive_ns"] is not None]
        lasts = [source["last_receive_ns"] for source in sources
                 if source["last_receive_ns"] is not None]
        origin = min(firsts) if firsts else None
        final = max(lasts) if lasts else None
        rows = []
        for (name, type_name, encoding), entries in sorted(topics.items()):
            if selected and name not in selected:
                continue
            count = 0
            begins, ends = [], []
            for connection, topic_id in entries:
                total, begin, end = connection.execute(
                    "SELECT COUNT(*), MIN(timestamp), MAX(timestamp) "
                    "FROM messages WHERE topic_id=?", (topic_id,)
                ).fetchone()
                count += total
                if begin is not None:
                    begins.append(begin)
                    ends.append(end)
            begin = min(begins) if begins else None
            end = max(ends) if ends else None
            row = {
                "name": name, "type": type_name, "serialization_format": encoding,
                "messages": count,
                "first_receive_ns": begin, "last_receive_ns": end,
                "first_relative_s": (begin - origin) * 1e-9 if begin is not None else None,
                "last_relative_s": (end - origin) * 1e-9 if end is not None else None,
                "mean_receive_hz_over_span": (
                    (count - 1) * 1e9 / (end - begin)
                    if count > 1 and end > begin else None
                ),
            }
            if timing:
                # topic IDs may differ between shards; merge timestamps, retaining duplicates.
                streams = [
                    (item[0] for item in connection.execute(
                        "SELECT timestamp FROM messages WHERE topic_id=? "
                        "ORDER BY timestamp,id", (topic_id,)
                    ))
                    for connection, topic_id in entries
                ]
                previous = None
                deltas = []
                duplicates = 0
                for stamp in heapq.merge(*streams):
                    if previous is not None:
                        gap = stamp - previous
                        duplicates += gap == 0
                        deltas.append(gap * 1e-9)
                    previous = stamp
                row["receive_interval_s"] = {
                    "count": len(deltas),
                    "min": min(deltas) if deltas else None,
                    "p50": percentile(deltas, 0.50),
                    "p95": percentile(deltas, 0.95),
                    "p99": percentile(deltas, 0.99),
                    "max": max(deltas) if deltas else None,
                    "zero_intervals": duplicates,
                }
            rows.append(row)
        types_by_name = defaultdict(set)
        for name, type_name, encoding in topics:
            types_by_name[name].add((type_name, encoding))
        return {
            "source": str(root),
            "scope": "目录中的全部 .db3" if root.is_dir() else "仅指定单分片",
            "storage": "sqlite3",
            "time_basis": "bag receive timestamp；不是 header/传感器时间",
            "quantiles": "线性插值；分片按时间归并，保留重复记录",
            "warnings": [
                "静态离线录制期间可复查；活跃写入中的 bag 不保证跨文件一致快照",
                "--timing 只保存时间间隔，不加载消息载荷；超长录制仍可能占用较多内存",
                "未验证 metadata 分片清单、QoS、header、掉帧、运行参数或内部状态",
            ],
            "type_conflicts": {
                name: sorted([list(value) for value in values])
                for name, values in types_by_name.items() if len(values) > 1
            },
            "first_receive_ns": origin, "last_receive_ns": final,
            "duration_s": (final - origin) * 1e-9 if origin is not None else None,
            "messages_all_shards": sum(source["messages"] for source in sources),
            "shards": sources, "topics": rows,
        }
    finally:
        for connection in connections:
            connection.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="bag 目录或单个 .db3 文件")
    parser.add_argument("--timing", action="store_true",
                        help="统计每个选定话题的接收间隔；可配合 --topic 缩小范围")
    parser.add_argument("--topic", action="append", default=[],
                        help="仅输出这个精确名称的话题；可重复指定")
    args = parser.parse_args()
    try:
        result = inspect(args.bag, args.timing, args.topic)
    except (ValueError, sqlite3.Error, OSError) as error:
        print(f"只读 bag 检查失败: {error}", file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
