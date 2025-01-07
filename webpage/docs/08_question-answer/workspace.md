# Automatically Source ROS 2 Workspace in New Terminal Sessions

To avoid having to source your ROS 2 workspace setup script every time you open a new terminal, you can add the source command to your `.bashrc` file. This file is executed every time you start a new terminal session, so adding the source command there will automatically set up your environment for you.

Here’s how you can do this:

1. **Open the `.bashrc` file**:
   You can open the `.bashrc` file using any text editor. For simplicity, we'll use `nano` in this example. Open a terminal and type:
   ```bash
   nano ~/.bashrc
   ```

2. **Add the source command**:
   Scroll to the bottom of the file and add the following line:
   ```bash
   source /home/cc/ros2_ws/install/setup.bash
   ```

3. **Save and exit**:
   - If you're using `nano`, save the file by pressing `Ctrl + O`, then press Enter to confirm. Exit `nano` by pressing `Ctrl + X`.
   - If you're using another editor like `vim`, after making the changes, press `Esc` to exit insert mode, then type `:wq` and press Enter to save and exit.

4. **Apply the changes**:
   To apply the changes you just made without having to open a new terminal, run:
   ```bash
   source ~/.bashrc
   ```

Now, every time you open a new terminal, the ROS 2 workspace setup script will be sourced automatically, and you won't have to do it manually each time.

Here's a summary of the steps in a single block for quick reference:

```bash
nano ~/.bashrc
# Add the following line at the end of the file:
source /home/cc/ros2_ws/install/setup.bash
# Save and exit
# For nano: Ctrl + O, Enter, Ctrl + X
# For vim: Esc, :wq, Enter
source ~/.bashrc
```

By following these steps, your environment will be set up automatically every time you open a new terminal.