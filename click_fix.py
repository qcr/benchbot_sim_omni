import subprocess
import sys
print("FIXING WEIRD CLICK BUG")
def install(package):
    subprocess.check_call([sys.executable, "-m", "pip", "install", package])
def uninstall(package):
    subprocess.check_call([sys.executable, "-m", "pip", "uninstall", "--yes", package])
uninstall("click")
install("click")
