#!/bin/bash

# 查找并删除所有包含 ':Zone.Identifier' 的文件
find . -type f -name "*:Zone.Identifier" -exec rm -f "{}" \;

echo "所有 ':Zone.Identifier' 文件已删除。"
