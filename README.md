## Code Versions
This repository contains two versions of the drive logic in the `/src` folder:
* **`drive_experimental.ino`**: The ideal, in-progress code. This version attempts to improve finish-line detection by reading the specific threshold of the middle 6 sensors individually.
* **`drive_working_backup.ino`**: The tested, stable code. This version uses a total sensor sum threshold (`> 8000`) to detect the finish line and successfully navigates the track.
