#pragma once

//Image Histogram
void DibHistogram(CDib& dib, float histo[256]);
//Image Equalization
void DibHistEqual(CDib& dib);
void Lo_DibHistEqual(CDib& dib,int blockSize,int th[]);
