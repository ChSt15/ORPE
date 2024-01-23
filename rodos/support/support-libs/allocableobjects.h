
/**
* @file allocableobjects.h
* @date 2021/02/11
* @author Sergio Montenegro
*
* @brief AllocableObject malloc/free with reference counter
*
*/


#pragma once
#include "stdint.h"
#include "rodos-result.h"

namespace RODOS {

/**
 * This base class is used to reference an AllocableObjects object without the length parameter.
 * @tparam Type The type of objects that can be allocated.
 */
template<typename Type>
class AllocableObjectsBase {
public:
  virtual ~AllocableObjectsBase() {}

  virtual Type *copyReference(Type *item) = 0;
  virtual bool free(Type *item) = 0;
};

/**
 * This is a smart pointer that automatically calls the reference counting mechanism of AllocableObjets.
 * @tparam Type The type of object that the shared pointer refers.
 */
template<typename Type>
class SharedPtr {
  /// Pointer to the pool from where the object was allocated. It is used for calling reference counting methods.
  AllocableObjectsBase<Type> *objectsPool = nullptr;

  Type *objectPointer = nullptr;  ///< The raw pointer to the object.

public:
  /**
   * Default constructor creates an invalid smart pointer, i.e. pointing to nullptr.
   */
  SharedPtr() = default;

  /**
   * Full constructor for creating a valid shared pointer. It is required that the objectPointer refers to an object
   * from the passed objectsPool.
   * @param objectsPool A pointer to the object pool containing the passed object.
   * @param objectPointer The raw pointer to the object that the shared pointer shall refer to.
   */
  SharedPtr(AllocableObjectsBase<Type> *objectsPool, Type *const objectPointer)
      : objectsPool(objectsPool), objectPointer(objectPointer) {}

  /**
   * Copy constructor for reference counting.
   * @param other The shared pointer from which the values should be copied from.
   */
  SharedPtr(const SharedPtr &other) : objectsPool(other.objectsPool), objectPointer(other.objectPointer) {
    if (objectsPool != nullptr) {
      objectsPool->copyReference(objectPointer);
    }
  }

  /**
   * Destructor for reference counting;
   */
  ~SharedPtr() {
    if (objectsPool != nullptr) {
      objectsPool->free(objectPointer);
    }
  }

  /**
   * Assigment operator for take new values and reference counting.
   * @param other The shared pointer from which the values should be copied from.
   * @return A reference to *this.
   */
  SharedPtr &operator=(const SharedPtr<Type> &other) {
    if (this != &other) {
      if (objectsPool != nullptr) {
        objectsPool->free(objectPointer);
      }
      objectsPool = other.objectsPool;
      objectPointer = other.objectPointer;
      if (objectsPool != nullptr) {
        objectsPool->copyReference(objectPointer);
      }
    }
    return *this;
  }

  /**
   * Dereference operator to access the object that this shared pointer refers to.
   * @return The raw pointer to the object that this shared pointer refers to.
   */
  Type *operator->() const {
    RODOS_ASSERT(isValid());
    return objectPointer;
  }

  /*
   * Dereference operator to access the object that this shared pointer refers to.
   * @return A reference to the object that this shared pointer refers to.
   */
  Type &operator*() const {
    RODOS_ASSERT(isValid());
    return *objectPointer;
  }

  /*
   * Clears the value of the shared pointer. After that, the shared pointer is invalid and refers to nullptr. The
   * method also accomplishes reference counting.
   */
  void clear() {
    if (objectsPool != nullptr) {
      objectsPool->free(objectPointer);
    }
    objectsPool = nullptr;
    objectPointer = nullptr;
  }

  /**
   * Checks whether the shared pointer is valid, i.e. refers to an object which is not nullptr and has a valid
   * pointer to the objects pool.
   * @return
   */
  bool isValid() const {
    return objectsPool != nullptr && objectPointer != nullptr;
  }

  /**
   * Retrieves the raw pointer to the referenced object.
   * @return the raw pointer to the referenced object.
   */
  Type *getRawPointer() const {
    return objectPointer;
  }
};

/**
 * Memory allocation with reference counter, for a "Reference Counting Garbage Collection"
 *
 * Each object has a count of the number of references to it.
 * object can be allocated with alloc. You can copy references ONLY by using the function copyReference
 * which will increment the reference counter.
 * the allocated object and copies of references shall be released using the function free.
 * when the reference counters reaches 0, free will put the object back to the free list.
 *
 * An object's reference count is incremented when a reference to it is created, and decremented when a reference is destroyed.
 * When the count reaches zero, the object's memory is reclaimed (not automatically, but when calling free)
 *
 * WARNING! Not Thread safe, if required protect with a semaphore
 *
 */

template<typename Type, uint32_t LENGTH>
class AllocableObjects : public AllocableObjectsBase<Type> {

  Type buffer[LENGTH];
  uint32_t referenceCnt[LENGTH]{};
  uint32_t freeCnt = LENGTH;

public:
  uint32_t getNumOfFreeItems() const { return freeCnt; }

  void init() {
    freeCnt = LENGTH;
    for (uint32_t i = 0; i < LENGTH; i++)
      referenceCnt[i] = 0;
  }

  /**
   * Allocates a new object from the memory pool.
   * @return A shared pointer to the object if there is memory left, otherwise it returns ErrorCode::Memory.
   */
  Result<SharedPtr<Type>> alloc() {
    for (uint32_t i = 0; i < LENGTH; i++) {
      if (referenceCnt[i] == 0) {
        freeCnt--;
        referenceCnt[i] = 1;
        return {{this, &buffer[i]}};
      }
    }
    return ErrorCode::MEMORY;
  }


  /**
   * Returns the index of the referenced item.
   * @param item The raw pointer to the item.
   * @return The index of the item within the memory pool, ErrorCode::BAD_POINTER otherwise.
   */
  Result<uint32_t> indexOf(Type *const item) {
    const uint32_t index = static_cast<uint32_t>(item - buffer);
    if (index > LENGTH)
      return ErrorCode::BAD_POINTER;
    return index;
  }

  /**
   * Returns the index of the referenced item.
   * @param item The shared pointer to the item.
   * @return The index of the item within the memory pool, ErrorCode::BAD_POINTER otherwise.
   */
  Result<uint32_t> indexOf(const SharedPtr<Type> &item) {
    return indexOf(item.getRawPointer());
  }

protected:
  bool free(Type *item) override {
    Result<uint32_t> index = indexOf(item);
    if (!index.isOk())
      return false;
    if (referenceCnt[index.val] < 1)
      return false;
    referenceCnt[index.val]--;
    if (referenceCnt[index.val] == 0)
      freeCnt++;
    return true;
  }

  // Returns pointer to the same item, or 0 if invalid
  Type *copyReference(Type *item) override {
    Result<uint32_t> index = indexOf(item);
    if (!index.isOk())
      return nullptr;
    if (referenceCnt[index.val] == 0)
      return nullptr;
    referenceCnt[index.val]++;
    return item;
  }

  /**
   * Only shared pointers are allowed to free objects or copy references.
   */
  friend class SharedPtr<Type>;
};
} // namespace RODOS
