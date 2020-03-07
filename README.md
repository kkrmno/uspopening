# uspopening

Author:
Teo Asplund

This is an implementation of the upper skeleton path opening as described
in the paper:

Asplund, Teo, and Cris L. Luengo Hendriks.
"A faster, Unbiased Path Opening by Upper Skeletonization and Weighted
Adjacency Graphs."
IEEE Transactions on Image Processing 25.12 (2016): 5589-5600.

Note that MATLAB and DIPImage are required to run this code.

# To compile:
---------------
In MATLAB, compile by

    mex pathOpeningUnbiased.cpp NodeClassNew.cpp

# To run:
---------------
After compilation, there should exist a mex-file called
"pathOpeningUnbiased.mex[...]" in the same directory as "uspopening.m".
First, make sure that DIPImage has been initialized (run "dipstart.m"),
then apply the opening to an 8-bit grayscale image, I, by

   uspopening(I, length, h, 1)

This will return a reconstructed opened image using the specified length
and h-value. If reconstruction is not desired, specify 0, instead of 1 as
the final parameter.

You may want to add a black border around your image as a preprocessing
step.

# Help:
---------------

   help uspopening

gives some information on how to apply the opening.
