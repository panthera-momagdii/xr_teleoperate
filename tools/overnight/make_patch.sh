#!/usr/bin/env bash
# Write ONE gate's patch as an INCREMENTAL diff against the previous gate's tree.
#
#   tools/overnight/make_patch.sh gNN_name <path> [<path> ...]
#
# Why not `git diff --cached`: that diffs against HEAD, so every gate's patch would
# re-contain every earlier gate's changes to the same file (the launcher is touched by
# G1, G2, G4 and G6). Applying them in order would conflict on the second one.
#
# Instead each gate's baseline is the TREE OBJECT produced by the previous gate, kept
# in patches/.tree_after_<gate>. No commits, no branches, no refs are created: a tree
# object is just content-addressed storage, invisible to `git log` and `git status`,
# and reachable only from the file we write here.
#
# Afterwards the index is reset, so the next gate starts clean while the working tree
# keeps every change.
set -euo pipefail

NAME="${1:?usage: make_patch.sh gNN_name <paths...>}"
shift
[ $# -gt 0 ] || { echo "no paths given" >&2; exit 2; }

ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT"
mkdir -p patches

# The most recent baseline, or HEAD's tree for the first gate.
BASE_FILE="$(ls -1 patches/.tree_after_* 2>/dev/null | sort | tail -1 || true)"
if [ -n "$BASE_FILE" ]; then
    BASE="$(cat "$BASE_FILE")"
    echo "baseline: $BASE  (from $BASE_FILE)"
else
    BASE="$(git rev-parse HEAD^{tree})"
    echo "baseline: $BASE  (HEAD tree -- first gate)"
fi

# Seed the index from the BASELINE, not from HEAD.
#
# This is the whole trick, and getting it wrong is silent and destructive: after a
# `git reset` the index holds HEAD's content, so an earlier gate's files look DELETED
# relative to the baseline and the patch happily includes those deletions. The first
# version of this script did exactly that -- g02 contained
# "delete file mode tools/cert_info.py" and reverted all of G1's docs, and the chain
# still "applied cleanly" because it was consistently wrong.
#
# read-tree writes only the index; the working tree is untouched.
git read-tree "$BASE"
git add -- "$@"

# --binary: without it a patch containing a binary file (G3 ships a .npz test fixture)
# is emitted as an unusable "Binary files differ" stub and `git apply` refuses it with
# "cannot apply binary patch ... without full index line". Harmless for text-only
# patches, so it is unconditional.
git diff --binary "$BASE" --cached > "patches/${NAME}.patch"

# This gate's full tree becomes the next gate's baseline.
NEW_TREE="$(git write-tree)"
echo "$NEW_TREE" > "patches/.tree_after_${NAME%%_*}"

# Back to a clean index that matches HEAD, as the ground rules require.
git reset --quiet
echo "wrote patches/${NAME}.patch  ($(wc -l < "patches/${NAME}.patch") lines)"
echo "next baseline: $NEW_TREE  -> patches/.tree_after_${NAME%%_*}"
echo
echo "--- files in this patch ---"
git diff --stat "$BASE" "$NEW_TREE"
