#!/bin/sh

SOURCE_LIST="$(find . -name '*.h' -or -name '*.hpp' -or -name '*.cpp' -or -name '*.cc')"

for file in $SOURCE_LIST; do
    printf "Formatting %s...\n" "$file"
    clang-format-3.9 -i -style=file "$file"
done
