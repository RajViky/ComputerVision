import yaml

w = 640
h = 480

matrix = [[1,2,3],
    [4,5,6],
    [7,8,9]]

fname = "/tmp/data.yaml"
with open(fname, "w") as f:
    yaml.dump({'camera_name': "usbTest"}, f)
    yaml.dump({'image_width': w}, f)
    yaml.dump({'image_height': h}, f)
    yaml.dump({'camera_matrix': matrix},f)