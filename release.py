import os
import datetime
import platform
from tqdm import tqdm
import py7zr

def compress_install_folder():
    # Get current date for the archive name
    current_date = datetime.datetime.now().strftime("%Y%m%d")
    
    # Get system architecture and platform info
    arch = platform.machine()
    os_platform = platform.system().lower()
    
    # Define source and destination paths
    source_folder = "install"
    archive_name = f"VSLAM_ws_{current_date}_{os_platform}_{arch}.7z"
    
    # Check if install folder exists
    if not os.path.exists(source_folder):
        print(f"Error: {source_folder} folder not found!")
        return False
    
    try:
        # Get total size of files to compress
        total_size = sum(os.path.getsize(os.path.join(dirpath, filename))
                        for dirpath, _, filenames in os.walk(source_folder)
                        for filename in filenames)
        
        # Initialize progress bar
        pbar = tqdm(total=total_size, unit='B', unit_scale=True, desc="Compressing")
        
        # Create 7z archive with progress callback
        with py7zr.SevenZipFile(archive_name, 'w') as archive:
            # Get all file paths
            for dirpath, _, filenames in os.walk(source_folder):
                for filename in filenames:
                    filepath = os.path.join(dirpath, filename)
                    arcname = os.path.relpath(filepath, source_folder)
                    # Update progress bar
                    file_size = os.path.getsize(filepath)
                    archive.write(filepath, arcname)
                    pbar.update(file_size)
        
        pbar.close()
        print(f"Successfully compressed {source_folder} to {archive_name}")
        return True
            
    except Exception as e:
        print(f"Error: {str(e)}")
        return False

if __name__ == "__main__":
    compress_install_folder()