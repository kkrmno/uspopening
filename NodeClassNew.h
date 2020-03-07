/*
 * Author:
 * Teo Asplund
 *
 * Reference: A faster, Unbiased Path Opening by Upper Skeletonization and 
 * Weighted Adjacency Graphs.
 *
 */

#ifndef NODECLASSNEW_H
#define NODECLASSNEW_H

#include <string>
#include <vector>
#include <queue>

#ifndef positiveMod
#define positiveMod(a,b) (a % b + b) % b
#endif

using namespace std;

class NodeClassNew
{
  public:
    NodeClassNew(unsigned char value, int row, int col);
    NodeClassNew();

    /* Value read from input image */
    void setValue(unsigned char value);
    unsigned char getValue();

    void resetValue();

    double getLength(int dir);
    void setLength(int dir, double length);

    int getRow();
    int getCol();

    bool isActive();
    void setActive();
    void setInactive();

    bool isEnqueued();
    void setEnqueued();
    void setNotEnqueued();
    
    bool getChangedTest();
    void setChangedTest();

    bool isChangedQueueFlag();
    void setChangedQueueFlag();
    void setNotChangedQueueFlag();

    double getAddedLength();
    void findCircularNeighbors_Test(vector<NodeClassNew*> *nodes_ptrs, int width, int height);

    bool hasNoNeighbors(int upDir);
    
    /* Offsets: N 1, NE 2, E 3, SE 4, S 5, SW 6, W 7, NW 0    % 8 */
    NodeClassNew *getCircNeighbor(int upDir, int whichOne);
    
    int getIndexInNodeList(int imageHeight);

  private:   
    double lengths[2];
    int row;
    int col;
    unsigned char value;
    unsigned char origValue;

    bool enqueued;
    bool changedQueueFlag;
    bool active;

    bool noNeighbors[8];

    int getCircIndex(int rowOff, int colOff);
    NodeClassNew *circNeighbors[8];

};

inline void NodeClassNew::resetValue()
{
  value = origValue;
  if(value > 0)
  {
    setActive();
    lengths[0] = 0.0;
    lengths[1] = 0.0;
  }
}

inline unsigned char NodeClassNew::getValue()
{
  return value;
}

inline void NodeClassNew::setValue(unsigned char val)
{
  value = val;
}


inline double NodeClassNew::getLength(int dir)
{
  return lengths[dir];
}

inline void NodeClassNew::setLength(int dir, double length)
{
  lengths[dir] = length;
}

inline int NodeClassNew::getRow()
{
  return row;
}

inline int NodeClassNew::getCol()
{
  return col;
}


//Flags
inline bool NodeClassNew::isActive()
{
  return active;
}

inline void NodeClassNew::setActive()
{
  active = true;
}

inline void NodeClassNew::setInactive()
{
  active = false;
}

inline bool NodeClassNew::isEnqueued()
{
  return enqueued;
}

inline void NodeClassNew::setEnqueued()
{
  enqueued = true;
}

inline void NodeClassNew::setNotEnqueued()
{
  enqueued = false;
}

inline bool NodeClassNew::isChangedQueueFlag()
{
  return changedQueueFlag;
}

inline void NodeClassNew::setChangedQueueFlag()
{
  changedQueueFlag = true;
}

inline void NodeClassNew::setNotChangedQueueFlag()
{
  changedQueueFlag = false;
}
//End of flags

inline double NodeClassNew::getAddedLength()
{
  return lengths[0] + lengths[1];
}

/* Does not hold after sorting (then points to index in output image).*/
inline int NodeClassNew::getIndexInNodeList(int imageHeight)
{
  return col*imageHeight + row;
}

/* upDir specifies the main direction of the cone. whichOne gives the element, 
   where 0 is the CCW element in the cone, 1 is the center element, and 2 is 
   the CW element */
inline NodeClassNew *NodeClassNew::getCircNeighbor(int upDir, int whichOne)
{
  return circNeighbors[positiveMod(upDir+(whichOne-1), 8)];
}

inline bool NodeClassNew::hasNoNeighbors(int upDir)
{
  return noNeighbors[positiveMod(upDir, 8)];
}

#endif // NODECLASSNEW_H
