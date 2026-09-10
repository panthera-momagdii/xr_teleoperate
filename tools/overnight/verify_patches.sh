#!/usr/bin/env bash
# Prove the patches apply cleanly, in order, to a pristine HEAD -- without touching
# the working tree, the index, or creating any commit/branch/ref.
#
# Uses a scratch GIT_INDEX_FILE seeded from HEAD and `git apply --cached`, so nothing
# is written outside the object store and the temporary index file.
set -uo pipefail
ROOT="$(git rev-parse --show-toplevel)"; cd "$ROOT"
IDX="$(mktemp -t verify_patches_idx.XXXXXX)"
trap 'rm -f "$IDX"' EXIT
export GIT_INDEX_FILE="$IDX"

git read-tree HEAD || { echo "read-tree failed"; exit 1; }
echo "seeded a scratch index from HEAD ($(git rev-parse --short HEAD))"
echo

fail=0
for p in $(ls -1 patches/g*.patch 2>/dev/null | sort); do
    printf '%-44s ' "$(basename "$p")"
    if git apply --cached --check "$p" 2>/tmp/verify_patches_err; then
        git apply --cached "$p" && echo "APPLIES CLEANLY"
    else
        echo "*** FAILED ***"
        sed 's/^/      /' /tmp/verify_patches_err
        fail=1
    fi
done
rm -f /tmp/verify_patches_err
echo
if [ "$fail" = 0 ]; then
    echo "resulting tree: $(git write-tree)"
    echo "ALL PATCHES APPLY IN ORDER"
else
    echo "SOME PATCHES DO NOT APPLY"
fi

# The strongest check: applying every patch to a pristine HEAD must reproduce, byte for
# byte, the working tree that produced them -- for every file the patches touch. A
# chain can "apply cleanly" and still be wrong (an early version of make_patch.sh
# produced a g02 that deleted all of g01's new files, and it applied cleanly).
if [ "$fail" = 0 ]; then
    echo "--- comparing the patched tree against the working tree ---"
    TOUCHED="$(for p in patches/g*.patch; do
                   grep -E '^\+\+\+ ' "$p" | sed 's|^+++ b/||' | grep -v '/dev/null'
               done | sort -u)"
    mismatch=0
    for f in $TOUCHED; do
        # `git diff` with no --cached is INDEX vs WORKING TREE, which with our scratch
        # GIT_INDEX_FILE means patched-tree vs working-tree. (`--cached` would compare
        # the index to HEAD instead, which is not the question.)
        if ! git diff --quiet -- "$f" 2>/dev/null; then
            echo "    DIFFERS from working tree: $f"
            mismatch=1
        fi
    done
    if [ "$mismatch" = 0 ]; then
        echo "    every patched file matches the working tree"
    else
        echo "    *** the patches do not reproduce the working tree ***"
        fail=1
    fi
    echo
fi

echo
echo "--- files each patch touches ---"
for p in $(ls -1 patches/g*.patch 2>/dev/null | sort); do
    echo "$(basename "$p"):"
    grep -E '^(\+\+\+|---) ' "$p" | sed 's|^+++ b/||;s|^--- a/||' | grep -v '/dev/null' \
        | sort -u | sed 's/^/    /'
done
exit $fail
