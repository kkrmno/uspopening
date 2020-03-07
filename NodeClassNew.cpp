/*
 * Author:
 * Teo Asplund
 *
 * Reference: A Faster, Unbiased Path Opening by Upper Skeletonization and 
 * Weighted Adjacency Graphs.
 *
 */

#include "NodeClassNew.h"
#include <stdexcept>
#include <iostream>
#include <queue>
#include <string>
#include <vector>

#include "mex.h"

using namespace std;

NodeClassNew::NodeClassNew(unsigned char val = 0, int r = 0, int c = 0)
{
  value = val;
  origValue = val;
  row = r;
  col = c;
  enqueued = false;
  changedQueueFlag = false;
  active = true;

  lengths[0] = lengths[1] = 0.0;

  for(int i = 0; i < 8; i++)
  {
      circNeighbors[i] = NULL;
      noNeighbors[i] = true;
  }
}

//Default constructor
NodeClassNew::NodeClassNew()
{
  value = 0;
  origValue = 0;
  row = 0;
  col = 0;
  enqueued = false;
  changedQueueFlag = false;
  active = true;

  lengths[0] = lengths[1] = 0.0;

  for(int i = 0; i < 8; i++)
  {
      circNeighbors[i] = NULL;
      noNeighbors[i] = true;
  }
}

/*
 *  0 1 2
 *  7 X 3
 *  6 5 4
 *
 */
int NodeClassNew::getCircIndex(int rowOff, int colOff)
{
    if(rowOff == -1)
    {
        return colOff + 1;
    }
    else if(rowOff == 0)
    {
        if(colOff == -1)
        {
            return 7;
        }
        else if(colOff == 1)
        {
            return 3;
        }
        else
        {
            throw runtime_error("Undefined circle index for these arguments.");
        }
    }
    else if(rowOff == 1)
    {
        if(colOff == -1)
        {
            return 6;
        }
        else if(colOff == 0)
        {
            return 5;
        }
        else if(colOff == 1)
        {
            return 4;
        }
    }
}


void NodeClassNew::findCircularNeighbors_Test(vector<NodeClassNew*> *nodes_ptrs, int width, int height)
{
    int neighborCol;
    int neighborRow;
    int neighborIndex;
    for(int cOff = -1; cOff < 2; cOff++)
    {
        neighborCol = col + cOff;
        for(int rOff = -1; rOff < 2; rOff++)
        {
            if(rOff == 0 && cOff == 0)
                continue;

            neighborRow = row + rOff;
            neighborIndex = neighborRow + neighborCol*height;

            /* Check neighbor is within bounds of image. */
            if(neighborCol >= 0 && neighborCol < width && neighborRow >= 0 && neighborRow < height)
            {
                NodeClassNew* node_ptr = (*nodes_ptrs)[neighborIndex];
                if(node_ptr && node_ptr->isActive())
                {
                  int circIndex = getCircIndex(rOff, cOff);
                  noNeighbors[positiveMod(circIndex-1, 8)] = false;
                  noNeighbors[circIndex] = false;
                  noNeighbors[positiveMod(circIndex+1, 8)] = false;
                  circNeighbors[circIndex] = node_ptr;
                }
            }
        }
    }
}
