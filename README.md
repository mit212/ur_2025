# Universal Robots UR5 Robot Arm - Example RTDE Code

2.12/2.120 Intro to Robotics  
Spring 2025[^1]

**_This code is provided for your convenience. Read through the comments in the files to understand how they work. If you have any questions, please ask the TAs._**

## Connecting to the UR5

If you have installed RTDE in Python directly (not on WSL), simply open your favorite text editor and run the code in Python. If you installed RTDE on Windows Subsystem for Linux (WSL), follow these steps:

1. Open a WSL terminal by entering `wsl` in Command Prompt. Your terminal should say something like `LAPTOP_NAME: mnt/c/Users/yourname$`. This is the current directory. 
2. Since the file you want to run is likely in another folder, we will change the current directory to be that folder. Open File Explorer and navigate to that folder.
3. Right-click the address bar at the top of the screen and select "Copy Address" from the dropdown that appears.

    <details> <summary> Where is the address bar? </summary>


    It is located to the left of the search bar. It should say something like "Documents > MIT > ur_2024".

    </details>

4. Go back to the WSL terminal and type `cd "`. Right-click to paste the address you copied and type `"` at the end. **Don't hit enter yet!**
5. Replace all the `\` with `/` and replace `C:` with `/mnt/c`. Your command should now look like 

    ```
    cd "/mnt/c/Users/yourname/Documents/MIT/ur_2025"
    ```

6. Hit enter. Your terminal should show a new current directory.
   <details> <summary> Directory not found error? </summary>

    Make sure you included the `/` before `mnt`. Also, if your original current directory had a different disk letter, make sure to use that instead of `c`, e.g. `/mnt/e`.
    </details>
7. Enter `python3 test_import.py`. You should see the success print! 

Note: In these steps, we navigated to our desired directory by entering its exact address. In the future, you may prefer to navigate incrementally via the terminal using the basic commands below.
- `ls`: returns the contents of your current directory
- `cd example`: goes to `example` subfolder **within the current directory**
- `cd ..`: goes to the folder that contains your current directory, think of this as going up by a level

[^1]: Version 1 - 2023: Ravi Tejwani, Erik Ballesteros, Chengyuan Ma, Kamal Youcef-Toumi  
  Version 2 - 2024: Jinger Chong  
  Version 3 - 2025: Roberto Bolli, Kaleb Blake
