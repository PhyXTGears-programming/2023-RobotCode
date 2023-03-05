#
# BLACK MAGIC CODE AHEAD
# BAD FORMATTING AS WELL
# made as a "works for now, will fix later (TM)"
# very verbose logging because I cannot be bothered to clean it up right now
# will clean up eventually
#
import json
import toml
import os

script_dir = os.path.dirname(__file__)

# toml_file = toml.load("{}/../src/main/deploy/config.toml".format(script_dir))["auto"]

# max_speed = toml_file["max_speed"]
max_speed = 4

#data = json.load("{}/../src/main/deploy/{}".format(script_dir, toml_file["pathplanner_file"]))
file = open("{}/test.json".format(script_dir), "r")
data = json.load(file)


def doNothing(): # in python you cannot have a try without a catch, this is here just do do nothing :)
    return
