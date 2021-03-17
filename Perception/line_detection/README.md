# Line Detection

Takes the semantic segmentation image, extracts roadmarks, detects lines with Hough-Transformation, clusters lines by angle.
To detecet stop lines, the following is done:

* Take the longest line in each cluster
* if 
    * there is a line with an angle smaller than a threshold (0.05 pi)
    * and the line is longer than threshold (0.35 imagewidth) 
* then take the its negative relative y-position as the relative distance to stop-line. 
* else publish infinity as distance.