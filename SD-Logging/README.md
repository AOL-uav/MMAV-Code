# Purdue AOL MAV - SD Logging Project

## Directory Structure
- **Firmware/**: Contains all Arduino sketches.
    - `gliding_pid_sd/`: The main flight controller sketch with SD logging.
    - `original_62_gliding_pid.ino`: The unmodified original code.
    - `mini_logger.ino`: The simple test logger used for verification.

## How to use the SD Files
The flight controller records data in standard **CSV (Comma Separated Values)** format. You can open these in two ways:

1.  **Via Serial Monitor (On the bench):**
    *   Open the Serial Monitor (115200 baud).
    *   Type **`r`** to dump the current log file text.
    *   Copy the text and save it as a `.csv` file on your PC.

2.  **Via SD Card (Best for flight data):**
    *   Remove the microSD card from the flight controller.
    *   Plug it into your PC using an SD adapter.
    *   The files are named `fl000.csv`, `fl001.csv`, etc.
    *   Open them directly with **Microsoft Excel**, **Google Sheets**, or any text editor.

## Analysis Tips
*   The first line of each file is the **Header**. It tells you what each column is.
*   The `ms` column is your timeline. Since we log at 100Hz, you should see about 10ms between each row.
*   If a file looks messy in a text editor, just open it in Excel and it will automatically format into clean columns.
