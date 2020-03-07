/*
 * pathOpeningUnbiased.cpp - performs graph based path opening. Used to
 * perform USPO.
 *
 * Opens an input image (uint8 matrix inputImage) with paths of length L
 * (double length) in four directions (S-N, SW-NE, W-E, NW-SE).
 * The output is a uint8 matrix (uint8 matrix outputImage) representing the
 * opened image.
 *
 * Note that the input image is expected to have a black border (i.e. value
 * 0).
 *
 * Calling syntax:
 *
 *      outputImage = pathOpeningUnbiased(inputImage, length)
 *
 * This is a MEX-file for MATLAB.
 *
 * Author:
 * Teo Asplund
 *
 * Reference: A Faster, Unbiased Path Opening by Upper Skeletonization and
 * Weighted Adjacency Graphs.
 *
 */


#include <queue>
#include <algorithm> //sort
#include <string>
#include <stdexcept>
#include <vector>
#include "NodeClassNew.h"

#include "mex.h"

// macro for checking error conditions
#ifndef errCheck
#define errCheck(a,b) if (!(a)) mexErrMsgTxt((b));
#endif

#ifndef positiveMod
#define positiveMod(a,b) (a % b + b) % b
#endif

// Unbiased weights, Kulpa (1977)
#define STRAIGHT_WEIGHT 0.948
#define DIAGONAL_WEIGHT 1.340

using namespace std;

bool nodeCompare(NodeClassNew & n1, NodeClassNew & n2)
{
  return n1.getValue() < n2.getValue();
}

bool nodePtrCompare(NodeClassNew *n1, NodeClassNew *n2)
{
  return n1->getValue() < n2->getValue();
}


/* Initializes up or down lengths from start_node in the graph. Uses unbiased weights.*/
inline void initializeFromNode(NodeClassNew *start_node, int up_or_down, int dir, double lengthThreshold)
{
  vector<NodeClassNew*> queue;
  queue.push_back(start_node);
  vector<NodeClassNew*> next_queue;
  int b_dir = positiveMod(dir + 4, 8);

  while(!queue.empty())
  {
    NodeClassNew *node = queue.back();
    queue.pop_back();
    node->setNotEnqueued();

    /* Find longest path looking backwards taking weights into account */
    double longestPath = 0.0;
    double pathLength = 0.0;
    for(int i = 0; i < 3; i++)
    {
      NodeClassNew *backNeighbor = node->getCircNeighbor(b_dir, i);

      if (!backNeighbor) continue;

      if((b_dir + i) % 2 != 0)
      {
        pathLength = backNeighbor->getLength(up_or_down) + DIAGONAL_WEIGHT;
      }
      else
      {
        pathLength = backNeighbor->getLength(up_or_down) + STRAIGHT_WEIGHT;
      }
      if(pathLength > longestPath)
      {
        longestPath = pathLength;
      }

      if(longestPath > lengthThreshold) break; //Small optimization
    }

    if(longestPath < 0.1)
    {
      longestPath = DIAGONAL_WEIGHT;
    }
    if(node->getLength(up_or_down) < longestPath)
    {
      node->setLength(up_or_down, longestPath);
      int i;
      for(i = 0; i < 3; i++)
      {
        NodeClassNew *downNeighbor = node->getCircNeighbor(dir, i);
        if (!downNeighbor) continue;
        if (!downNeighbor->isEnqueued()) next_queue.push_back(downNeighbor);
        downNeighbor->setEnqueued();
      }
    }
    if(queue.size() == 0)
    {
      queue = next_queue;
      next_queue.clear();
    }
  }
}

/*
 * creates the graph and initializes length values at the lowest threshold
 * nodes with value 0 are set to inactive
 */
void createGraph_Test(vector<NodeClassNew> *nodes, unsigned char *image, size_t width, size_t height)
{
  size_t numberOfNodes = 0;
  size_t imageSize = width*height;

  size_t row = 0;
  size_t col = 0;

  vector<NodeClassNew*> nodes_ptrs;


  for(int i = 0; i < imageSize; i++)
  {
    col = i / height;
    row = i - col*height;

    /* Set values of the nodes to the corresponding image value */
    if(image[i] > 0)
    {
        numberOfNodes++;
        (*nodes).push_back(NodeClassNew(image[i], row, col)); //Maybe use (*nodes).emplace_back(...) instead
        nodes_ptrs.push_back(&((*nodes).back())); //Push the address of the created node onto the nodes_ptrs vector. CAREFUL! POINTERS ARE INVALIDATED IF NODES VECTOR ALLOCATES MORE MEMORY!
    }
    else
    {
        nodes_ptrs.push_back(NULL);
    }

  }

  /* Connect active nodes to each other according to adjacency relation. */
  for(int i = 0; i < numberOfNodes; i++)
  {
      (*nodes)[i].findCircularNeighbors_Test(&nodes_ptrs, width, height);
  }
}

/* Initializes up and down lengths for all active nodes. Uses unbiased weights */
void initializeLengths(vector<NodeClassNew> *nodes, int startIndex, int dir, int up_or_down, unsigned char *image, int height, double lengthThreshold)
{
  int b_dir = positiveMod(dir + 4, 8);
  for(int i = startIndex; i < (*nodes).size(); i++)
  {
    //errCheck((*nodes)[i]->getValue() != 0, "All nodes should have non-zero value!");
    NodeClassNew nd = (*nodes)[i];
    if(nd.isActive())
    {
      if(nd.hasNoNeighbors(b_dir))
      {
        initializeFromNode(&((*nodes)[i]), 1-up_or_down, dir, lengthThreshold);
      }
      if(nd.hasNoNeighbors(dir))
      {
        initializeFromNode(&((*nodes)[i]), up_or_down, b_dir, lengthThreshold);
      }
    }
  }
}

/* Enqueues node in fifo_queue iff node is active */
inline void enqueue_active(NodeClassNew *node_ptr, queue<NodeClassNew*> *fifo_queue)
{
    if(node_ptr->isActive() && !node_ptr->isEnqueued())
    {
        fifo_queue->push(node_ptr);
        node_ptr->setEnqueued();
    }
}

/* Enqueues node in fifo_changed iff node is active */
inline void enqueue_changed(NodeClassNew *node_ptr, queue<NodeClassNew*> *fifo_changed)
{
    if(node_ptr->isActive() && !node_ptr->isChangedQueueFlag())
    {
        fifo_changed->push(node_ptr);
        node_ptr->setChangedQueueFlag();
    }
}


inline void propagate(NodeClassNew *node_ptr, int up_or_down, int dir, queue<NodeClassNew*> *fifo_changed, queue<NodeClassNew*> *fifo_queue, double lengthThreshold)
{
    int b_dir = positiveMod(dir + 4, 8);
    node_ptr->setLength(up_or_down, 0);

    /* Enqueue all active forward (up) neighbors */
    for(int i = 0; i < 3; i++)
    {
        NodeClassNew *upNeighbor = node_ptr->getCircNeighbor(dir, i);

        if (!upNeighbor) continue;
        enqueue_active(upNeighbor, fifo_queue);
    }

    while(fifo_queue->size() > 0)
    {

        NodeClassNew *fwd_node_ptr = fifo_queue->front();
        fifo_queue->pop();

        fwd_node_ptr->setNotEnqueued();

        /* Find longest path looking backwards (down) taking weights into account */
        double longestPath = 0.0;
        double pathLength = 0.0;
        for(int i = 0; i < 3; i++)
        {
          NodeClassNew *backNeighbor = fwd_node_ptr->getCircNeighbor(b_dir, i);

          if (!backNeighbor) continue;

          if((b_dir + i) % 2 == 0)
          {
            pathLength = backNeighbor->getLength(up_or_down) + STRAIGHT_WEIGHT;
          }
          else
          {
            pathLength = backNeighbor->getLength(up_or_down) + DIAGONAL_WEIGHT;
          }
          if(pathLength > longestPath)
          {
            longestPath = pathLength;
          }

          if(longestPath > lengthThreshold) break; //Small optimization
        }

        /* This happens when looking at an end pixel. Should possibly use
         * special weight in this case. */
        if(longestPath < 0.1)
        {
          longestPath = DIAGONAL_WEIGHT;
        }

        /* Update backwards path if length has decreased */  //This part is a small optimization
        if(longestPath < fwd_node_ptr->getLength(up_or_down) && longestPath < lengthThreshold)
        {
            fwd_node_ptr->setLength(up_or_down, longestPath);


            //Enqueue in Q all active neighbors f
            for(int i = 0; i<3; i++)
            {
                NodeClassNew *upNeighbor = fwd_node_ptr->getCircNeighbor(dir, i);

                if (!upNeighbor) continue;
                enqueue_active(upNeighbor, fifo_queue);
            }

            //Enqueue in Q_c if active
            enqueue_changed(fwd_node_ptr, fifo_changed);

        }
    }

}


/*
 * Performs directed path opening on a vector of sorted node pointers.
 * The nodes of value > 0 start at index startOfVector
 * Uses unbiased weights.
 */
void directedPathOpening(vector<NodeClassNew*> *sorted_nodes, double length, size_t startOfVector, int dir)
{
    /*
     * Set nodes in too short paths to inactive (this is after length init.)
     *
     */
    for(int i = startOfVector; i < sorted_nodes->size(); i++)
    {
        NodeClassNew* nd_ptr = (*sorted_nodes)[i];
        if(nd_ptr->getAddedLength() >= length)
        {
            nd_ptr->setActive();
        }
        else
        {
            nd_ptr->setInactive();
            nd_ptr->setValue(0);
        }
    }

    int b_dir = positiveMod(dir + 4, 8);
    queue<NodeClassNew*> fifo_changed;
    queue<NodeClassNew*> fifo_queue;
    NodeClassNew *node_ptr;

    /* Propagate lengths as nodes "drop out" when threshold increases */
    for(int i = startOfVector; i < sorted_nodes->size(); i++)
    {
        node_ptr = (*sorted_nodes)[i];
        if(node_ptr->isActive())
        {
            propagate(node_ptr, 0, b_dir, &fifo_changed, &fifo_queue, length);
            propagate(node_ptr, 1, dir, &fifo_changed, &fifo_queue, length);

            /* Check changed nodes to see if they are not part of long
             * enough path anymore */
            while(fifo_changed.size() > 0)
            {
                NodeClassNew *changed_node_ptr = fifo_changed.front();
                fifo_changed.pop();
                changed_node_ptr->setNotChangedQueueFlag();
                if(changed_node_ptr->getAddedLength() < length)
                {
                    changed_node_ptr->setValue(node_ptr->getValue());
                    changed_node_ptr->setInactive();
                }
            }
            node_ptr->setInactive();
        }
    }
}

/* Takes a vector of nodes and an image height and outputs an image with
 * pixels having values corresponding to node values */
void graphToImage(unsigned char *output, vector<NodeClassNew> *nodes, size_t height)
{
    size_t i;

    for(i=0; i<nodes->size(); i++)
    {
        output[(*nodes)[i].getIndexInNodeList(height)] = (*nodes)[i].getValue();
    }
}

/* Takes a vector of nodes and an image height and outputs an image with
 * pixels having values corresponding to node values */
void graphToMaxImage(unsigned char *output, vector<NodeClassNew> *nodes, size_t height)
{
    size_t i;

    for(i=0; i<nodes->size(); i++)
    {
      int index = (*nodes)[i].getIndexInNodeList(height);
      int val = (*nodes)[i].getValue();
      if(output[index] < val)
      {
        output[index] = val;
      }
      (*nodes)[i].resetValue();
    }
}

size_t findFirstNonZeroValueNode(vector<NodeClassNew*> *sorted_nodes)
{
    size_t i = 0;
    while((*sorted_nodes)[i]->getValue() == 0)
    {
        i++;
    }
    return i;
}

/* Gateway function
 *
 * nlhs - number of output arguments (i.e. size of plhs array)
 * plhs - array of output arguments
 * nrhs - number of input arguments (i.e. size of prhs array)
 * prhs - array of input arguments
 *
 */
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    errCheck(nrhs == 2, "Two inputs required.");
    errCheck(nlhs == 1, "One output required.");
    errCheck(mxIsUint8(prhs[0]), "Input image must be of type uint8.");
    errCheck(mxIsDouble(prhs[1]), "Length must be of type double.");
    errCheck(mxGetNumberOfElements(prhs[1]) == 1, "Length must be a scalar.");

    double length; /* Length of opening */
    unsigned char *inputImage;
    size_t height, width;     /* image dimensions */
    unsigned char *outputImage;

    vector<NodeClassNew> nodes;
    vector<NodeClassNew*> nodes_ptrs;


    /* Get input from MATLAB */
    length = mxGetScalar(prhs[1]);
    inputImage = (unsigned char *)mxGetData(prhs[0]);

    height = mxGetM(prhs[0]);
    width  = mxGetN(prhs[0]);

    /* Stops unnecessary reallocation */

    /*
     * DO NOT REMOVE!
     * POINTERS INVALIDATED IF NODES VECTOR ALLOCATES MORE MEMORY!
     * AS LONG AS ENOUGH SPACE IS AVAILABLE IN NODES VECTOR, POINTERS
     * ARE GUARANTEED TO BE CORRECT! Reserving width*height space
     * guarantees no reallocation.
     */
    //============================================================
                nodes.reserve(width*height);
    //============================================================
    nodes_ptrs.reserve(width*height);

    /* Create output matrix */
    plhs[0] = mxCreateNumericMatrix(height, width, mxUINT8_CLASS, mxREAL);
    /* Get pointer to data in output matrix */
    outputImage = (unsigned char *)mxGetData(plhs[0]);

    createGraph_Test(&nodes, inputImage, width, height);

    for(int i=0; i<nodes.size(); i++)
    {
      nodes_ptrs.push_back(&(nodes[i]));
    }
    sort(nodes_ptrs.begin(), nodes_ptrs.end(), nodePtrCompare);


    /* Perform directed path opening on the nodes of value > 0 */
    /* directionOffset goes from 1 to 4 i.e. N to SE. */
    for(int directionOffset=1; directionOffset<=4; directionOffset++)
    {
      initializeLengths(&nodes, 0, positiveMod(directionOffset+4, 8), 1, inputImage, height, length);
      directedPathOpening(&nodes_ptrs, length, 0, directionOffset);
      graphToMaxImage(outputImage, &nodes, height);
    }

}
