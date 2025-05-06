import pandas as pd

def transform_old_dataset(input_csv, output_csv):
    # Load the old dataset
    df = pd.read_csv(input_csv)

    if not all(col in df.columns for col in ['extent_x', 'extent_y', 'aspect_ratio', 'area', 'num_points', 'label']):
        raise ValueError("Input CSV does not have the expected columns.")

    new_data = []

    for idx, row in df.iterrows():
        extent_x = row['extent_x']
        extent_y = row['extent_y']
        aspect_ratio = extent_x / extent_y if extent_y != 0 else 0.0
        area = extent_x * extent_y
        num_points = row['num_points']

        # Calculate the new features
        compactness = 0.0  # No cluster points available, so default to 0 (⚡ will improve with new data)
        elongation = extent_x / extent_y if extent_y != 0 else 0.0
        density = num_points / area if area != 0 else 0.0

        label = row['label']

        # Append all features in correct order
        new_data.append([
            extent_x,
            extent_y,
            aspect_ratio,
            area,
            num_points,
            compactness,
            elongation,
            density,
            label
        ])

    # Define new columns
    columns = ['extent_x', 'extent_y', 'aspect_ratio', 'area', 'num_points', 'compactness', 'elongation', 'density', 'label']

    # Save the new dataset
    new_df = pd.DataFrame(new_data, columns=columns)
    new_df.to_csv(output_csv, index=False)
    print(f"✅ Transformed dataset saved to {output_csv}")

if __name__ == '__main__':
    input_csv = '/home/jarred/git/drive_to_survive/src/cluster_puck/training_data/cluster_training_data.csv'  # Your old file
    output_csv = '/home/jarred/git/drive_to_survive/src/cluster_puck/training_data/cluster_training_data_v2.csv'  # New file
    transform_old_dataset(input_csv, output_csv)
