Import("env")
import os
import subprocess


def create_git_ref_files():
    # Get the current git commit hash using git command
    try:
        git_hash = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], 
            cwd=env.subst("$PROJECT_DIR"),
            stderr=subprocess.DEVNULL,
            text=True
        ).strip()
        print(f"Found git hash: {git_hash}")
    except:
        # Fallback if git command fails
        git_hash = "0000000000000000000000000000000000000000"
        print("Could not get git hash, using dummy value")
    
    # Create head-ref for main build
    build_dir = os.path.join(env.subst("$BUILD_DIR"), "CMakeFiles", "git-data")
    os.makedirs(build_dir, exist_ok=True)
    head_ref_file = os.path.join(build_dir, "head-ref")
    with open(head_ref_file, 'w') as f:
        f.write(git_hash + "\n")
    print(f"Created head-ref at {head_ref_file}")
    
    # Create head-ref for bootloader
    bootloader_dir = os.path.join(env.subst("$BUILD_DIR"), "bootloader", "CMakeFiles", "git-data")
    os.makedirs(bootloader_dir, exist_ok=True)
    bootloader_ref = os.path.join(bootloader_dir, "head-ref")
    with open(bootloader_ref, 'w') as f:
        f.write(git_hash + "\n")
    print(f"Created bootloader head-ref at {bootloader_ref}")


create_git_ref_files()