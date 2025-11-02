#!/usr/bin/env python3
"""
Script to inspect pickle files and print data shapes and structures
Works with both navigation and locomotion output files
"""

import pickle
import numpy as np
from pathlib import Path
import argparse
from typing import Any, Dict, List, Optional


def print_shapes(obj: Any, name: str = "root", depth: int = 0, max_depth: int = 3):
    """
    Print shape information for any Python object
    
    Args:
        obj: Object to inspect
        name: Name/label for this object
        depth: Current recursion depth
        max_depth: Maximum recursion depth
    """
    indent = "  " * depth
    
    if isinstance(obj, dict):
        print(f"{indent}{name}: Dict with {len(obj)} keys")
        if depth < max_depth:
            for key, value in obj.items():
                print_shapes(value, key, depth + 1, max_depth)
    
    elif isinstance(obj, list):
        if len(obj) == 0:
            print(f"{indent}{name}: List[0] (empty)")
        else:
            print(f"{indent}{name}: List[{len(obj)}]")
            if depth < max_depth:
                print_shapes(obj[0], f"{name}[0]", depth + 1, max_depth)
    
    elif isinstance(obj, np.ndarray):
        print(f"{indent}{name}: ndarray{obj.shape} (dtype={obj.dtype})")
    
    elif isinstance(obj, (int, float, str, bool)) or obj is None:
        val_str = str(obj) if obj is not None else "None"
        if len(val_str) > 50:
            val_str = val_str[:47] + "..."
        print(f"{indent}{name}: {type(obj).__name__} = {val_str}")
    
    else:
        print(f"{indent}{name}: {type(obj).__name__}")


def _print_dict_statistics(data: Dict, skip_keys: Optional[List[str]] = None):
    """Print statistics for dictionary data"""
    if skip_keys is None:
        skip_keys = []
    
    print(f"\n{'='*70}")
    print("Data Statistics:")
    print(f"{'='*70}")
    
    # Print special keys first if they exist
    if 'data_counts' in data:
        print(f"\ndata_counts:")
        for key, value in data['data_counts'].items():
            print(f"  {key}: {value}")
    
    # Print structure for main data fields
    for key, value in data.items():
        if key in skip_keys:
            continue
        
        print(f"\n{key}:")
        if isinstance(value, list):
            print(f"  Length: {len(value)}")
            if len(value) > 0:
                if isinstance(value[0], dict):
                    print(f"  First entry keys: {list(value[0].keys())}")
                    # Show nested structure details
                    sample = value[0]
                    for k, v in list(sample.items())[:3]:
                        if isinstance(v, dict):
                            print(f"    {k}: dict with keys {list(v.keys())}")
                        elif isinstance(v, list):
                            print(f"    {k}: list[{len(v)}]")
                            if len(v) > 0:
                                if isinstance(v[0], dict):
                                    print(f"      First item keys: {list(v[0].keys())}")
                                else:
                                    print(f"      First item type: {type(v[0]).__name__}")
                        else:
                            print(f"    {k}: {type(v).__name__}")
        elif isinstance(value, dict):
            print(f"  Dict with {len(value)} keys: {list(value.keys())}")
        elif isinstance(value, str):
            print(f"  Value: {value}")
        else:
            print(f"  Type: {type(value).__name__}")


def inspect_navigation_file(file_path: Path):
    """Inspect navigation bag output files"""
    print(f"\n{'='*70}")
    print(f"File: {file_path.name}")
    print(f"{'='*70}")
    
    with open(file_path, 'rb') as f:
        data = pickle.load(f)
    
    print(f"\nType: {type(data).__name__}")
    print_shapes(data, "data", max_depth=3)
    
    if isinstance(data, dict):
        _print_dict_statistics(data)


def inspect_locomotion_dataset(file_path: Path):
    """Inspect locomotion dataset file"""
    print(f"\n{'='*70}")
    print(f"File: {file_path.name}")
    print(f"{'='*70}")
    
    with open(file_path, 'rb') as f:
        data = pickle.load(f)
    
    print(f"\nType: {type(data).__name__}")
    print_shapes(data, "dataset", max_depth=2)
    
    if isinstance(data, dict):
        _print_dict_statistics(data, skip_keys=['data_counts'])


def main():
    parser = argparse.ArgumentParser(
        description="Inspect pickle files and print data shapes/structures"
    )
    parser.add_argument(
        "file_path",
        type=str,
        help="Path to pickle file or directory containing pickle files"
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Inspect all pickle files in directory"
    )
    
    args = parser.parse_args()
    
    file_path = Path(args.file_path)
    
    if file_path.is_dir() and args.all:
        # Find all pickle files
        pkl_files = list(file_path.glob("*.pkl"))
        if not pkl_files:
            print(f"No pickle files found in {file_path}")
            return
        
        print(f"Found {len(pkl_files)} pickle files:")
        for pkl_file in sorted(pkl_files):
            inspect_file(pkl_file)
    
    elif file_path.is_file():
        inspect_file(file_path)
    
    elif file_path.is_dir():
        # Try to find common output files
        common_files = [
            "locomotion_dataset.pkl",
            "navigation_data_structured.pkl",
            "imu_data.pkl",
            "lowstate_data.pkl",
            "odometry_data.pkl",
            "controller_data.pkl",
            "lowcmd_data.pkl"
        ]
        
        found_files = []
        for filename in common_files:
            potential_file = file_path / filename
            if potential_file.exists():
                found_files.append(potential_file)
        
        if found_files:
            print(f"Found {len(found_files)} output files:")
            for f in found_files:
                inspect_file(f)
        else:
            print(f"No common output files found in {file_path}")
            print("Use --all to inspect all pickle files")
    
    else:
        print(f"Error: {file_path} does not exist")
        return 1
    
    return 0


def inspect_file(file_path: Path):
    """Inspect a single pickle file"""
    try:
        if file_path.name == "locomotion_dataset.pkl":
            inspect_locomotion_dataset(file_path)
        else:
            inspect_navigation_file(file_path)
    except Exception as e:
        print(f"\nError reading {file_path}: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    exit(main())

