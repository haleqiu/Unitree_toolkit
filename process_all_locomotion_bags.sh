#!/bin/bash

# Simplified script to process all locomotion bag sequences
# Processes all sequences in the locomotion collection directory

# Configuration
COLLECTION_DIR="/home/ANT.AMAZON.COM/yuhengq/data/locomotion_collection"
SCRIPT_DIR="/home/ANT.AMAZON.COM/yuhengq/workspace/Unitree_toolkit"
PYTHON_SCRIPT="$SCRIPT_DIR/read_locomotion_bag_ros2.py"
OUTPUT_BASE_DIR="$COLLECTION_DIR/new_extracted"
SAVE_IMAGES="${SAVE_IMAGES:-true}"  # Set SAVE_IMAGES=false to skip images

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Simple print functions
info() { echo -e "${GREEN}[INFO]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
err() { echo -e "${RED}[ERROR]${NC} $1"; }

# Validate inputs
[ ! -d "$COLLECTION_DIR" ] && err "Collection directory not found: $COLLECTION_DIR" && exit 1
[ ! -f "$PYTHON_SCRIPT" ] && err "Python script not found: $PYTHON_SCRIPT" && exit 1

cd "$COLLECTION_DIR" || exit 1
mkdir -p "$OUTPUT_BASE_DIR"

# Find all unique timestamps
declare -A timestamps
for dir in navigation_bag_*; do
    [[ -d "$dir" && "$dir" =~ navigation_bag_([0-9]{8}_[0-9]{6})_.* ]] && \
        timestamps["${BASH_REMATCH[1]}"]=1
done

[ ${#timestamps[@]} -eq 0 ] && warn "No sequences found" && exit 0

info "Found ${#timestamps[@]} sequence(s) to process"

# Process each sequence
processed=0
skipped=0
failed=0

for timestamp in $(printf '%s\n' "${!timestamps[@]}" | sort); do
    seq_name="navigation_bag_${timestamp}"
    output_dir="$OUTPUT_BASE_DIR/$seq_name"
    temp_dir="$COLLECTION_DIR/temp_$timestamp"
    
    echo ""
    info "Processing: $timestamp"
    
    # Skip if already processed
    if [[ -d "$output_dir" && -f "$output_dir/locomotion_dataset.pkl" ]]; then
        warn "Already processed: $seq_name"
        ((skipped++))
        continue
    fi
    
    # Find bag directories for this timestamp
    bag_dirs=()
    for suffix in livox other zed; do
        bag_dir="$COLLECTION_DIR/navigation_bag_${timestamp}_${suffix}"
        [[ -d "$bag_dir" ]] && bag_dirs+=("$bag_dir")
    done
    
    [[ ${#bag_dirs[@]} -eq 0 ]] && err "No bags found for $timestamp" && ((failed++)) && continue
    
    # Create temp directory with symlinks
    mkdir -p "$temp_dir"
    for bag_dir in "${bag_dirs[@]}"; do
        ln -sf "$bag_dir" "$temp_dir/$(basename "$bag_dir")"
    done
    
    # Build command
    cmd="/usr/bin/python3 '$PYTHON_SCRIPT' --collection_path '$temp_dir' --output '$output_dir'"
    [[ "$SAVE_IMAGES" == "false" ]] && cmd="$cmd --no-images"
    
    # Run processing
    if bash -c "source /opt/ros/humble/setup.bash && source ~/unitree_ros2/install/setup.bash && $cmd"; then
        info "✓ Success: $seq_name"
        ((processed++))
    else
        err "✗ Failed: $seq_name"
        [[ -d "$output_dir" ]] && rm -rf "$output_dir"
        ((failed++))
    fi
    
    rm -rf "$temp_dir"
done

# Summary
echo ""
info "=== Summary ==="
info "Processed: $processed | Skipped: $skipped | Failed: $failed"
[[ $processed -gt 0 ]] && info "Output: $OUTPUT_BASE_DIR"

exit 0
