# import the necessary packages
from PIL import Image
import pytesseract
import argparse
import cv2
import os
from pathlib import Path

replacements = [
        [ '$', '9' ],
        [ '(', '' ],
        [ ')', '' ],
        [ 'O', '0' ],
        [ '{', '' ],
        [ '}', '' ]
    ]

def main():
    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", required=False, help="path to input image to be OCR'd")
    ap.add_argument("-d", "--dir", required=False, help="walk through folder, its subdirs are taken as class names")
    ap.add_argument("-p", "--preprocess", type=str, default="tb", help="type of preprocessing to be done (t = thresh, b = blur, tb = both)")
    ap.add_argument("-r", "--resize_to", type=int, default=0, help="resize to width X, preserving the aspect ratio")
    args = vars(ap.parse_args())
    
    if args["image"]:
        image = preprocessing(args["image"], args["resize_to"], args["preprocess"])
        read_ocr(image)
    elif args["dir"]:
        count = 0
        correct = 0
        for subdir, dirs, files in os.walk(args["dir"]):
            classname = subdir.split('/')[-1]
            if not classname.startswith('traffic_sign'):
                continue
            for file in files:
                image = preprocessing(os.path.abspath(os.path.join(subdir, file)), args["resize_to"], args["preprocess"])
                recognitions = read_ocr(image, psms=[13]) # 8,13
                recognized_number = ''.join([ i for i in recognitions[0].split() if i.isdigit() ])
                expected = classname[-2:]
                print("testing {}, expected output {}, got {}".format(os.path.join(classname, file), expected, recognized_number))
                
                count += 1
                if expected in recognized_number:
                    correct += 1
                
        print("tested {} files, {} were correct ({} %)".format(count, correct, round(correct/count*100, 2)))
    
    
def preprocessing(image, resize_to, preprocess):
    # load the example image and convert it to grayscale
    image = cv2.imread(image)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    if resize_to != 0:
        resize_factor = resize_to / gray.shape[1]
        new_width = int(gray.shape[1] * resize_factor)
        new_height = int(gray.shape[0] * resize_factor)
        gray = cv2.resize(gray, (new_width, new_height))
    # check to see if we should apply thresholding to preprocess the
    # image
    
    if "b" in preprocess:
    	gray = cv2.medianBlur(gray, 3)
    if "t" in preprocess:
    	#gray = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
    	gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 1)
    	#ADAPTIVE_THRESH_GAUSSIAN_C
    	
    # make a check to see if median blurring should be done to remove
    # noise
    # write the grayscale image to disk as a temporary file so we can
    # apply OCR to it
    
    #filename = "gray.png".format(os.getpid())
    #cv2.imwrite(filename, gray)
    cv2.imshow("test", gray)
    cv2.waitKey(100)
    return gray


def read_ocr(image, psms=False, oem=False):
    config="--psm {} --oem {}"
    
    if oem == False:
        oem = 1
        
    if psms != False and type(psms) != list:
        psms = [psms]
    elif psms == False:
        psms = range(3, 14)

    recognitions = []
    for psm in psms:
        text = ""
        try:
            text = pytesseract.image_to_string(image, config=config.format(psm, oem), lang='eng')
            text = text.strip()
            for replacement in replacements:
                text = text.replace(replacement[0], replacement[1])
            #print("psm {}, oem {}: {}".format(psm, oem, text))
            recognitions.append(text)
        except Exception as e:
            print("psm {}, oem {}, Exception: ".format(psm, oem), e)
            pass
            
    return recognitions
        
       
if __name__ == "__main__":
    main()
