#!/usr/bin/env python3
"""
Compare MESH and CTS sink locations
- Load both CSV files
- Match sinks by name
- Calculate differences
- Generate scatter plots and statistics
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def load_sinks(filename):
    """Load sink data from CSV file"""
    if not os.path.exists(filename):
        print(f"ERROR: File not found: {filename}")
        return None

    df = pd.read_csv(filename)
    print(f"Loaded {len(df)} sinks from {filename}")
    return df

def compare_sinks(mesh_df, cts_df):
    """Compare sink locations between MESH and CTS"""

    # Extract instance names (before the '/')
    mesh_df['inst'] = mesh_df['name'].str.split('/').str[0]
    cts_df['inst'] = cts_df['name'].str.split('/').str[0]

    # Merge on instance name
    merged = pd.merge(mesh_df, cts_df, on='inst', how='outer',
                      suffixes=('_mesh', '_cts'), indicator=True)

    # Statistics
    both = merged[merged['_merge'] == 'both']
    mesh_only = merged[merged['_merge'] == 'left_only']
    cts_only = merged[merged['_merge'] == 'right_only']

    print("\n" + "="*60)
    print("SINK COMPARISON STATISTICS")
    print("="*60)
    print(f"Total MESH sinks:     {len(mesh_df)}")
    print(f"Total CTS sinks:      {len(cts_df)}")
    print(f"Sinks in both:        {len(both)}")
    print(f"Sinks only in MESH:   {len(mesh_only)}")
    print(f"Sinks only in CTS:    {len(cts_only)}")

    if len(both) > 0:
        # Calculate position differences for matching sinks
        both['x_diff'] = abs(both['x_mesh'] - both['x_cts'])
        both['y_diff'] = abs(both['y_mesh'] - both['y_cts'])
        both['dist'] = np.sqrt(both['x_diff']**2 + both['y_diff']**2)

        print("\n" + "="*60)
        print("POSITION DIFFERENCES (for matching sinks)")
        print("="*60)
        print(f"Max X difference:     {both['x_diff'].max():.2f} DBU")
        print(f"Max Y difference:     {both['y_diff'].max():.2f} DBU")
        print(f"Max distance:         {both['dist'].max():.2f} DBU")
        print(f"Mean X difference:    {both['x_diff'].mean():.2f} DBU")
        print(f"Mean Y difference:    {both['y_diff'].mean():.2f} DBU")
        print(f"Mean distance:        {both['dist'].mean():.2f} DBU")

        # Check if positions match exactly
        exact_match = len(both[both['dist'] == 0])
        print(f"\nExact position matches: {exact_match}/{len(both)}")

        if exact_match == len(both):
            print("✓ SUCCESS: All sink positions match perfectly!")
        elif both['dist'].max() < 10:
            print("✓ SUCCESS: All differences < 10 DBU (likely rounding)")
        else:
            print("⚠ WARNING: Some positions differ significantly")

        # Show worst mismatches
        if both['dist'].max() > 0:
            print("\nTop 5 worst position mismatches:")
            worst = both.nlargest(5, 'dist')[['inst', 'x_mesh', 'y_mesh', 'x_cts', 'y_cts', 'dist']]
            print(worst.to_string(index=False))

    # Show MESH-only sinks
    if len(mesh_only) > 0:
        print("\n" + "="*60)
        print("SINKS FOUND BY MESH BUT NOT CTS:")
        print("="*60)
        print(mesh_only[['name_mesh', 'x_mesh', 'y_mesh', 'net_mesh']].head(10).to_string(index=False))
        if len(mesh_only) > 10:
            print(f"... and {len(mesh_only) - 10} more")

    # Show CTS-only sinks
    if len(cts_only) > 0:
        print("\n" + "="*60)
        print("SINKS FOUND BY CTS BUT NOT MESH:")
        print("="*60)
        print(cts_only[['name_cts', 'x_cts', 'y_cts', 'net_cts']].head(10).to_string(index=False))
        if len(cts_only) > 10:
            print(f"... and {len(cts_only) - 10} more")

    print("="*60 + "\n")

    return merged, both

def plot_sinks(mesh_df, cts_df, both_df, output_dir):
    """Generate visualization plots"""

    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(16, 14))

    # Plot 1: Overlay scatter plot
    ax1 = axes[0, 0]
    ax1.scatter(mesh_df['x'], mesh_df['y'], c='blue', alpha=0.5, s=20, label='MESH', marker='o')
    ax1.scatter(cts_df['x'], cts_df['y'], c='red', alpha=0.5, s=20, label='CTS', marker='x')
    ax1.set_xlabel('X (DBU)')
    ax1.set_ylabel('Y (DBU)')
    ax1.set_title('Sink Locations: MESH vs CTS (Overlay)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Plot 2: Side-by-side
    ax2 = axes[0, 1]
    ax2.scatter(mesh_df['x'], mesh_df['y'], c='blue', alpha=0.6, s=30, label=f'MESH ({len(mesh_df)} sinks)')
    ax2.set_xlabel('X (DBU)')
    ax2.set_ylabel('Y (DBU)')
    ax2.set_title('MESH Sink Locations')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # Plot 3: CTS only
    ax3 = axes[1, 0]
    ax3.scatter(cts_df['x'], cts_df['y'], c='red', alpha=0.6, s=30, label=f'CTS ({len(cts_df)} sinks)')
    ax3.set_xlabel('X (DBU)')
    ax3.set_ylabel('Y (DBU)')
    ax3.set_title('CTS Sink Locations')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Plot 4: Position differences (if matching sinks exist)
    ax4 = axes[1, 1]
    if len(both_df) > 0 and 'dist' in both_df.columns:
        # Histogram of distances
        ax4.hist(both_df['dist'], bins=30, color='green', alpha=0.7, edgecolor='black')
        ax4.set_xlabel('Position Difference (DBU)')
        ax4.set_ylabel('Number of Sinks')
        ax4.set_title(f'Position Differences for {len(both_df)} Matching Sinks')
        ax4.axvline(both_df['dist'].mean(), color='red', linestyle='--', label=f'Mean: {both_df["dist"].mean():.2f}')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
    else:
        ax4.text(0.5, 0.5, 'No matching sinks to compare',
                ha='center', va='center', transform=ax4.transAxes)
        ax4.set_title('Position Differences')

    plt.tight_layout()

    # Save plot
    output_file = os.path.join(output_dir, 'sink_comparison.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {output_file}")

    # Create a second figure showing arrows for position differences
    if len(both_df) > 0 and both_df['dist'].max() > 0:
        fig2, ax = plt.subplots(figsize=(12, 10))

        # Plot MESH positions
        ax.scatter(both_df['x_mesh'], both_df['y_mesh'], c='blue', s=50,
                  alpha=0.6, label='MESH', zorder=3)

        # Plot CTS positions
        ax.scatter(both_df['x_cts'], both_df['y_cts'], c='red', s=50,
                  alpha=0.6, label='CTS', marker='x', zorder=3)

        # Draw arrows from MESH to CTS for sinks with differences
        diff_sinks = both_df[both_df['dist'] > 0]
        for _, sink in diff_sinks.head(50).iterrows():  # Show max 50 arrows
            ax.arrow(sink['x_mesh'], sink['y_mesh'],
                    sink['x_cts'] - sink['x_mesh'],
                    sink['y_cts'] - sink['y_mesh'],
                    head_width=200, head_length=300, fc='gray', ec='gray',
                    alpha=0.3, zorder=1)

        ax.set_xlabel('X (DBU)')
        ax.set_ylabel('Y (DBU)')
        ax.set_title('Position Differences (arrows show MESH→CTS shift)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        output_file2 = os.path.join(output_dir, 'sink_differences.png')
        plt.savefig(output_file2, dpi=150, bbox_inches='tight')
        print(f"Difference plot saved to: {output_file2}")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))

    mesh_file = os.path.join(script_dir, 'mesh_sinks.csv')
    cts_file = os.path.join(script_dir, 'cts_sinks.csv')

    print("="*60)
    print("MESH vs CTS Sink Location Comparison")
    print("="*60)

    # Load data
    mesh_df = load_sinks(mesh_file)
    cts_df = load_sinks(cts_file)

    if mesh_df is None or cts_df is None:
        print("\nERROR: Could not load sink data files")
        print("Please run the extraction scripts first:")
        print("  1. cd /home/wajid/OpenROAD/build")
        print("  2. ./bin/openroad ../src/mesh/test/test_sinks/extract_mesh_sinks.tcl")
        print("  3. ./bin/openroad ../src/mesh/test/test_sinks/run_cts_with_logging.tcl")
        print("Or run: cd ../src/mesh/test/test_sinks && make")
        return 1

    # Compare
    merged, both = compare_sinks(mesh_df, cts_df)

    # Plot
    plot_sinks(mesh_df, cts_df, both, script_dir)

    # Save merged data
    output_file = os.path.join(script_dir, 'comparison_results.csv')
    merged.to_csv(output_file, index=False)
    print(f"\nDetailed comparison saved to: {output_file}")

    return 0

if __name__ == '__main__':
    sys.exit(main())
