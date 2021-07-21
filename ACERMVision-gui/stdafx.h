#ifndef STDAFX_H_INCLUDE
#define STDAFX_H_INCLUDE

#define OPENCV_VERSION 0400

#define LINUX   0x01
#define WINDOWS 0x02
#define MACOS   0x03

//#ifndef GUI
//#define CUI
//#endif

//#if defined(GUI) || defined(CUI)
//#define DEBUG   //necessary
//#elif   //defined(GUI) || defined(CUI)
//#define DEBUG
//#endif  //defined(GUI) || defined(CUI)


#ifdef DEBUG

#endif

//能量机关宏开关
#define BUFF_DETECT_DEBUG

#define TYPE_ARMOR_CLASSIFIER 2	//装甲分类器类型，0:svm 1:dnn 2:lenet

#endif	//STDAFX_H_INCLUDE
