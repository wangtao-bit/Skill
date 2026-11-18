#!/bin/sh
set -eu
base=$(CDPATH='' cd -P "$(dirname "$0")" && pwd)
src="$base/local-freecad-design"
dest="${HOME:?}/.agents/skills/local-freecad-design"
test -r "$src/SKILL.md" && test -r "$src/references/f330-case.md" || { echo '技能文件不全，请重新解压。' >&2; exit 1; }
mkdir -p "$(dirname "$dest")"
mkdir "$dest" || { echo "未安装：请检查目录是否已存在或不可写：$dest" >&2; exit 1; }
cp -R "$src/." "$dest/"
printf '安装完成：%s\n' "$dest"
