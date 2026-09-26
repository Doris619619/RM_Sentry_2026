#!/usr/bin/env python3
"""Bound pixels rendered by this launch's two visible windows without moving the mouse."""
import os
import re
import shutil
import subprocess
import time
import psutil

# Resize only descendants of this ROS launch, leaving every unrelated desktop window untouched.
def main():
    if not shutil.which('wmctrl') or not os.environ.get('DISPLAY'):
        print('Optional GUI layout unavailable; arrange Gazebo and RViz side by side manually.',flush=True)
        return
    launch_pid=os.getppid()
    root_info=subprocess.check_output(['xwininfo','-root'],text=True)
    screen_width=int(re.search(r'Width: (\d+)',root_info).group(1))
    screen_height=int(re.search(r'Height: (\d+)',root_info).group(1))
    width=min(1200,max(500,(screen_width-100)//2))
    height=min(850,max(400,screen_height-220))
    done=set()
    deadline=time.monotonic()+30
    while time.monotonic()<deadline and len(done)<2:
        for line in subprocess.check_output(['wmctrl','-lpG'],text=True).splitlines():
            parts=line.split(None,8)
            if len(parts)<9:continue
            xid,pid,title=parts[0],int(parts[2]),parts[8]
            role='gazebo' if title=='Gazebo' else 'rviz' if title.endswith(' - RViz') else None
            if role is None or role in done:continue
            try:
                if launch_pid not in [p.pid for p in psutil.Process(pid).parents()]:continue
            except psutil.Error:continue
            x=60+(width+10 if role=='rviz' else 0)
            subprocess.run(['wmctrl','-ir',xid,'-b','remove,maximized_vert,maximized_horz'],check=True)
            subprocess.run(['wmctrl','-ir',xid,'-e',f'0,{x},100,{width},{height}'],check=True)
            done.add(role)
        time.sleep(.25)
    print('Visible simulation windows arranged: '+','.join(sorted(done)),flush=True)

if __name__=='__main__':main()
