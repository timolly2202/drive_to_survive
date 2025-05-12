import json

# List of cone quartets: (yellow1, yellow2, blue1, blue2)
cone_quartets = [
    ("yellow_cone_17", "yellow_cone_18", "blue_cone_28", "blue_cone_29"),
    ("yellow_cone_13", "yellow_cone_14", "blue_cone_32", "blue_cone_43"),
    ("yellow_cone_10", "yellow_cone_11", "blue_cone_34", "blue_cone_35"),
    ("yellow_cone_6", "yellow_cone_7", "blue_cone_37", "blue_cone_38"),
    ("yellow_cone_3", "yellow_cone_4", "blue_cone_40", "blue_cone_41"),
    ("yellow_cone_46", "yellow_cone_47", "blue_cone_1", "blue_cone_2"),
    ("yellow_cone_42", "yellow_cone_43", "blue_cone_5", "blue_cone_6"),
    ("yellow_cone_39", "yellow_cone_40", "blue_cone_8", "blue_cone_45"),
    ("yellow_cone_36", "yellow_cone_37", "blue_cone_9", "blue_cone_10"),
    ("yellow_cone_0", "yellow_cone_34", "blue_cone_12", "blue_cone_13"),
    ("yellow_cone_31", "yellow_cone_32", "blue_cone_15", "blue_cone_16"),
    ("yellow_cone_28", "yellow_cone_29", "blue_cone_18", "blue_cone_47"),
    ("yellow_cone_25", "yellow_cone_26", "blue_cone_20", "blue_cone_21"),
    ("yellow_cone_22", "yellow_cone_23", "blue_cone_22", "blue_cone_23"),
    ("yellow_cone_21", "yellow_cone_48", "blue_cone_24", "blue_cone_25"),
    ("yellow_cone_19", "yellow_cone_20", "blue_cone_26", "blue_cone_27")
]

# Load the cone centres
with open("/home/jarred/git/drive_to_survive/validation/ground_truths/cone_centers.json", "r") as file:
    data = json.load(file)

# Build a lookup for quick access
cone_lookup = {cone["name"]: (cone["x"], cone["y"]) for cone in data["cones"]}

goals = []

# Loop through cone quartets
for i, (c1, c2, c3, c4) in enumerate(cone_quartets):
    try:
        x1, y1 = cone_lookup[c1]
        x2, y2 = cone_lookup[c2]
        x3, y3 = cone_lookup[c3]
        x4, y4 = cone_lookup[c4]

        # Calculate the centre
        centre_x = (x1 + x2 + x3 + x4) / 4
        centre_y = (y1 + y2 + y3 + y4) / 4

        goals.append({
            "goal_id": i,
            "centre": {"x": centre_x, "y": centre_y},
            "cones": [
                {"name": c1, "x": x1, "y": y1},
                {"name": c2, "x": x2, "y": y2},
                {"name": c3, "x": x3, "y": y3},
                {"name": c4, "x": x4, "y": y4}
            ]
        })
    except KeyError as e:
        print(f"Error: Cone {e} not found in data.")

# Save the result
with open("/home/jarred/git/drive_to_survive/src/rl_rewards/goals/goals.json", "w") as out_file:
    json.dump({"goals": goals}, out_file, indent=2)

print("Goals JSON created successfully.")
