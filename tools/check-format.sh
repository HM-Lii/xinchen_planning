#!/usr/bin/env bash

set -euo pipefail

readonly base_ref="${1:-HEAD}"
readonly formatter="${CLANG_FORMAT:-clang-format}"

if ! command -v git-clang-format >/dev/null 2>&1; then
  echo "git-clang-format is required for format-check" >&2
  exit 2
fi
if ! command -v "${formatter}" >/dev/null 2>&1; then
  echo "${formatter} is required for format-check" >&2
  exit 2
fi
if ! git rev-parse --verify "${base_ref}^{commit}" >/dev/null 2>&1; then
  echo "format base revision does not exist: ${base_ref}" >&2
  exit 2
fi

readonly format_diff="$(
  git-clang-format --binary "${formatter}" --diff "${base_ref}" -- \
    'src/*.cpp' 'src/*.h' 'tests/*.cpp'
)"

case "${format_diff}" in
  'no modified files to format'|'clang-format did not modify any files')
    exit 0
    ;;
esac

echo "Changed C++ lines do not match .clang-format:" >&2
echo "${format_diff}" >&2
exit 1
