import os

def compare_folders(folder1, folder2):
    """
    Compares two folders to check if the files (excluding extensions) are matching.
    Prints names of files that do not match.

    Parameters:
    - folder1: First folder containing images.
    - folder2: Second folder containing files with extensions (e.g. .txt files).
    """
    # Get list of file names without extensions in both folders
    folder1_files = [os.path.splitext(f)[0] for f in os.listdir(folder1) if os.path.isfile(os.path.join(folder1, f))]
    folder2_files = [os.path.splitext(f)[0] for f in os.listdir(folder2) if os.path.isfile(os.path.join(folder2, f))]

    # Check for unmatched files in folder1
    unmatched_folder1 = set(folder1_files) - set(folder2_files)
    if unmatched_folder1:
        print("Files in folder 1 not found in folder 2:")
        for file in unmatched_folder1:
            print(file)

    # Check for unmatched files in folder2
    unmatched_folder2 = set(folder2_files) - set(folder1_files)
    if unmatched_folder2:
        print("Files in folder 2 not found in folder 1:")
        for file in unmatched_folder2:
            print(file)

    # If all files match, print a success message
    if not unmatched_folder1 and not unmatched_folder2:
        print("All files match between the two folders.")

def main():
    # Take input for two folder paths
    folder1 = "/home/road2022/Documents/aaaa/rokey_on_site/my_data/valid/images"
    folder2 = "/home/road2022/Documents/aaaa/rokey_on_site/my_data/valid/labels"

    # Check if both folders exist
    if not os.path.exists(folder1):
        print(f"Error: Folder '{folder1}' does not exist.")
        return
    if not os.path.exists(folder2):
        print(f"Error: Folder '{folder2}' does not exist.")
        return

    # Compare the two folders
    compare_folders(folder1, folder2)

if __name__ == "__main__":
    main()
