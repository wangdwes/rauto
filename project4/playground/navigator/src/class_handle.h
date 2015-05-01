
/* Copyright (c) 2014-2015, Team Evolutus
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

// completely adopted from this package.  
// http://www.mathworks.com/matlabcentral/fileexchange/38964-example-matlab-class-wrapper-for-a-c++-class

#ifndef CLASS_HANDLE_
#define CLASS_HANDLE_

#include <mex.h>
#include <stdint.h>
#include <string>
#include <cstring>
#include <typeinfo>

// crc32 for "evl::Navigator". 
#define CLASS_HANDLE_SIGNATURE 0x4086b552 

template <class Class> 
class ClassHandle
{
public: 

  /** \brief Construct the class and save some meta-information. */ 
  ClassHandle(Class* pointer): 
    pointer_(pointer),
    type_name_(typeid(Class).name()),
    signature_(CLASS_HANDLE_SIGNATURE) {}

  /** \brief Erase the signature and release storage space. */ 
  ~ClassHandle() { signature_ = 0; delete pointer_; } 

  /** \brief Retrieve the pointer. */ 
  Class *pointer() { return pointer_; } 

  /** \brief Check validity by comparing the signature and the type name. */ 
  bool isValid() { 
    return (signature_ == CLASS_HANDLE_SIGNATURE) &&
           !strcmp(type_name_.c_str(), typeid(Class).name()); }  

protected:
  uint32_t signature_;      //!< an identifying signature for this handler.  
  std::string type_name_;   //!< a string that describes this type. 
  Class *pointer_;          //!< a pointer to the class instance.

};

// ideally these functions should be encapsulated in some class
// as static member functions. but for simplicity we leave them as-is.  

template <class Class>
inline mxArray* convertPtr2Mat(Class *pointer)
{
  mexLock(); // prevent the .mex from being cleared from the memory.
  mxArray *mat = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL); // create the pointer as a scalar. 
  *((uint64_t*)mxGetData(mat)) = reinterpret_cast<uint64_t>(new ClassHandle<evl::Navigator>(pointer)); // yeah... 
  
  return mat; 
}

template <class Class>
inline ClassHandle<Class> *convertMatToHandlePtr(const mxArray *mat)
{
  // just to make sure that no one deliberately attempts to sabotage our effort. 
  if (mxGetNumberOfElements(mat) != 1 || mxGetClassID(mat) != mxUINT64_CLASS || mxIsComplex(mat))
    mexErrMsgTxt("Input must be a real uint64 scalar."); 
  
  // convert the numerical value back to a pointer.  
  ClassHandle<Class> *pointer = reinterpret_cast<ClassHandle<Class>*>(*((uint64_t*)mxGetData(mat)));
  if (!pointer->isValid()) mexErrMsgTxt("Handle is not valid.");
  return pointer; 
} 

template <class Class>
inline Class *convertMat2Ptr(const mxArray *mat)
{
  return convertMatToHandlePtr<Class>(mat)->pointer(); 
} 

template <class Class>
inline void destroyObject(const mxArray *mat) 
{
  // retrieve the pointer and unlock the .mex from being deleted. 
  delete convertMatToHandlePtr<Class>(mat);
  mexUnlock();  
} 

#endif // CLASS_HANDLE_

