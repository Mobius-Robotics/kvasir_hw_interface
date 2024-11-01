#!/usr/bin/env python3

import os
import shutil

# Define old and new package names
OLD_PACKAGE_NAME = "loki_hw_interface"
NEW_PACKAGE_NAME = "loki_hw_interface"

# Define old and new class names
OLD_CLASS_NAME = "LokiHardware"
NEW_CLASS_NAME = "LokiHardware"

# Define other terms to replace
OLD_ROBOT_NAME = "loki"
NEW_ROBOT_NAME = "loki"

def snake_to_camel(snake_str):
    return ''.join(word.title() for word in snake_str.split('_'))

# Prepare variations of the names for replacements
OLD_PACKAGE_NAME_UPPER = OLD_PACKAGE_NAME.upper()
NEW_PACKAGE_NAME_UPPER = NEW_PACKAGE_NAME.upper()

OLD_PACKAGE_NAME_CAMEL = snake_to_camel(OLD_PACKAGE_NAME)
NEW_PACKAGE_NAME_CAMEL = snake_to_camel(NEW_PACKAGE_NAME)

OLD_ROBOT_NAME_UPPER = OLD_ROBOT_NAME.upper()
NEW_ROBOT_NAME_UPPER = NEW_ROBOT_NAME.upper()

OLD_ROBOT_NAME_CAMEL = snake_to_camel(OLD_ROBOT_NAME)
NEW_ROBOT_NAME_CAMEL = snake_to_camel(NEW_ROBOT_NAME)

print(f"Renaming project from '{OLD_PACKAGE_NAME}' to '{NEW_PACKAGE_NAME}'...")
print(f"Renaming class from '{OLD_CLASS_NAME}' to '{NEW_CLASS_NAME}'...")
print(f"Replacing robot name from '{OLD_ROBOT_NAME}' to '{NEW_ROBOT_NAME}'...")

# Step 1: Replace text in files
for root, dirs, files in os.walk('.'):
    for file in files:
        file_path = os.path.join(root, file)
        # Skip binary files
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            content_new = content

            # Replace package names
            content_new = content_new.replace(OLD_PACKAGE_NAME, NEW_PACKAGE_NAME)
            content_new = content_new.replace(OLD_PACKAGE_NAME_UPPER, NEW_PACKAGE_NAME_UPPER)
            content_new = content_new.replace(OLD_PACKAGE_NAME_CAMEL, NEW_PACKAGE_NAME_CAMEL)

            # Replace class names
            content_new = content_new.replace(OLD_CLASS_NAME, NEW_CLASS_NAME)

            # Replace robot names
            content_new = content_new.replace(OLD_ROBOT_NAME, NEW_ROBOT_NAME)
            content_new = content_new.replace(OLD_ROBOT_NAME_UPPER, NEW_ROBOT_NAME_UPPER)
            content_new = content_new.replace(OLD_ROBOT_NAME_CAMEL, NEW_ROBOT_NAME_CAMEL)

            if content != content_new:
                with open(file_path, 'w', encoding='utf-8') as f:
                    f.write(content_new)
                print(f"Updated file: {file_path}")
        except UnicodeDecodeError:
            continue  # Skip non-text files

# Step 2: Rename files and directories
for root, dirs, files in os.walk('.', topdown=False):
    # Rename files
    for name in files:
        old_path = os.path.join(root, name)
        new_name = name

        if any(term in name for term in [
            OLD_PACKAGE_NAME, OLD_PACKAGE_NAME_UPPER, OLD_PACKAGE_NAME_CAMEL,
            OLD_CLASS_NAME,
            OLD_ROBOT_NAME, OLD_ROBOT_NAME_UPPER, OLD_ROBOT_NAME_CAMEL
        ]):
            new_name = new_name.replace(OLD_PACKAGE_NAME, NEW_PACKAGE_NAME)
            new_name = new_name.replace(OLD_PACKAGE_NAME_UPPER, NEW_PACKAGE_NAME_UPPER)
            new_name = new_name.replace(OLD_PACKAGE_NAME_CAMEL, NEW_PACKAGE_NAME_CAMEL)

            new_name = new_name.replace(OLD_CLASS_NAME, NEW_CLASS_NAME)

            new_name = new_name.replace(OLD_ROBOT_NAME, NEW_ROBOT_NAME)
            new_name = new_name.replace(OLD_ROBOT_NAME_UPPER, NEW_ROBOT_NAME_UPPER)
            new_name = new_name.replace(OLD_ROBOT_NAME_CAMEL, NEW_ROBOT_NAME_CAMEL)

            new_path = os.path.join(root, new_name)
            shutil.move(old_path, new_path)
            print(f"Renamed file: {old_path} -> {new_path}")

    # Rename directories
    for name in dirs:
        old_path = os.path.join(root, name)
        new_name = name

        if any(term in name for term in [
            OLD_PACKAGE_NAME, OLD_PACKAGE_NAME_UPPER, OLD_PACKAGE_NAME_CAMEL,
            OLD_CLASS_NAME,
            OLD_ROBOT_NAME, OLD_ROBOT_NAME_UPPER, OLD_ROBOT_NAME_CAMEL
        ]):
            new_name = new_name.replace(OLD_PACKAGE_NAME, NEW_PACKAGE_NAME)
            new_name = new_name.replace(OLD_PACKAGE_NAME_UPPER, NEW_PACKAGE_NAME_UPPER)
            new_name = new_name.replace(OLD_PACKAGE_NAME_CAMEL, NEW_PACKAGE_NAME_CAMEL)

            new_name = new_name.replace(OLD_CLASS_NAME, NEW_CLASS_NAME)

            new_name = new_name.replace(OLD_ROBOT_NAME, NEW_ROBOT_NAME)
            new_name = new_name.replace(OLD_ROBOT_NAME_UPPER, NEW_ROBOT_NAME_UPPER)
            new_name = new_name.replace(OLD_ROBOT_NAME_CAMEL, NEW_ROBOT_NAME_CAMEL)

            new_path = os.path.join(root, new_name)
            shutil.move(old_path, new_path)
            print(f"Renamed directory: {old_path} -> {new_path}")

print("Renaming completed successfully.")

