#!/bin/bash

INPLACE=false
ROOT_DIR=$(git rev-parse --show-toplevel)

if [[ "$1" == "-i" ]]; then
    INPLACE=true
fi

FILES=$(find "$ROOT_DIR" -type f \( -name "*.cpp" -o -name "*.h" \))

if $INPLACE; then
    echo "Formatting files in-place..."
    clang-format -i $FILES
    echo "Done."
    exit 0
else
    echo "Running clang-format in dry-run mode..."

    HAS_ERRORS=0
    for FILE in $FILES; do
        DIFF=$(clang-format --dry-run --Werror "$FILE" 2>&1)
        if [[ $? -ne 0 ]]; then
            echo "Formatting issue in: $FILE"
            echo "$DIFF"
            HAS_ERRORS=1
        fi
    done

    if [[ $HAS_ERRORS -eq 1 ]]; then
        echo "Some files do not meet clang-format standards."
        exit 1
    else
        echo "All files are properly formatted."
        exit 0
    fi
fi
