# Type help("robodk.robolink") or help("robodk.robomath") for more information
# Press F5 to run the script
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/robodk.html
# Note: It is not required to keep a copy of this file, your Python script is saved with your RDK project

# You can also use the new version of the API:
from robodk import robolink    # RoboDK API
from robodk import robomath    # Robot toolbox
RDK = robolink.Robolink()

# Forward and backwards compatible use of the RoboDK API:
# Remove these 2 lines to follow python programming guidelines
from robodk import *      # RoboDK API
from robolink import *    # Robot toolbox
# Link to RoboDK
# RDK = Robolink()

# Notify user:
print('To edit this program:\nright click on the Python program, then, select "Edit Python script"')

# Program example:
item = RDK.Item('base')
if item.Valid():
    print('Item selected: ' + item.Name())
    print('Item posistion: ' + repr(item.Pose()))

print('Items in the station:')
itemlist = RDK.ItemList()
print(itemlist)

# Provoking an exception will display the console output if you run this script from RoboDK
#raise Exception('Program not edited.')