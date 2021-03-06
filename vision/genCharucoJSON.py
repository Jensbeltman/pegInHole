import argparse
import json

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generates JSON with charucoBoard parameters")
    parser.add_argument('squaresX',help="squaresX parameter used for OpenCv charuco board initialization")
    parser.add_argument('squaresY',help="squaresY parameter used for OpenCv charuco board initialization")
    parser.add_argument('markerLength',help="markerLength parameter used for OpenCv charuco board initialization")
    parser.add_argument('squareLength',help="squareLength parameter used for OpenCv charuco board initialization")
    parser.add_argument('dictionary',help="dictionary parameter used for OpenCv charuco board initialization")
    parser.add_argument("--output_file",nargs="?",default='./json/charucoBoard.json',help="output file")

    args = parser.parse_args()

    params = {'squaresX':int(args.squaresX),'squaresY':int(args.squaresY),'markerLength':float(args.markerLength),'squareLength':float(args.squareLength),'dictionary':args.dictionary}

    with open(args.output_file, 'w') as outfile:
            json.dump(params, outfile,indent=4)



