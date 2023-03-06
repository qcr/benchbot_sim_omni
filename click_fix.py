import subprocess
import sys
print("HACK FIX FOR CLICK BUG")
def install(package):
    subprocess.check_call([sys.executable, "-m", "pip", "install", package])
def uninstall(package):
    subprocess.check_call([sys.executable, "-m", "pip", "uninstall", "--yes", package])
uninstall("click")
install("click")
uninstall("typing-extensions")
install("typing-extensions")
print("NOW WITH ALL NEW TYPING EXTENSIONS")

