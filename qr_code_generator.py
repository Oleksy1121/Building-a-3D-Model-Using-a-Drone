### simple script to generate QR code with requested parameters for the project
### please use qr_code_generator.py --help for usage


import json
import qrcode
import sys, argparse

parser = argparse.ArgumentParser()
parser.add_argument("-s", "--marker_size", help="size of marker in cm (4-30)", type=int)
parser.add_argument("-d", "--marker_distance", help="initial distance from marker in cm (10, 50)", type=int)
parser.add_argument("-r", "--radius", help="radius around object in cm (50, 100)", type=int)
parser.add_argument("-z", "--heights", help="comma separated list of orbits heights in cm (50-150)", type=str)
args = None
try:
    args = parser.parse_args()
    print(args)
    if args.marker_size not in range(4, 31):
        print("marker size must be in range (4, 30)!")
        parser.print_help()
        exit(0)
    if args.marker_distance not in range(10, 51):
        print("marker distance must be in range (10, 50)!")
        parser.print_help()
        exit(0)
    if args.radius not in range(50, 101):
        print("radius must be in range (50, 100)!")
        parser.print_help()
        exit(0)
    for h in args.heights.split(","):
        if int(h) < 50 or int(h) > 150:
            print("height must be in range (50-150)!")
            parser.print_help()
            exit(0)
except Exception as e:
    print(e)
    parser.print_help()
    exit(0)

dict = {
    "preamble": "IiSRL__TELLO_LAB_L2_G5",
    "data": {
        "marker_size": args.marker_size,
        "marker_distance": args.marker_distance,
        "radius": args.radius,
        "heights": [int(h) for h in args.heights.split(",")]
    }
}


code = qrcode.QRCode(
    error_correction=qrcode.constants.ERROR_CORRECT_L,
    box_size=10,
    border=2
)
code.add_data(json.dumps(dict))
code.make()

img = code.make_image()
img.save('tello_qr_code.png')

print("QR code ready")
print(json.dumps(dict, indent=3))
