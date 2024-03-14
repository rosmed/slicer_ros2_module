#ifndef vtkCustomTypes_h
#define vtkCustomTypes_h

#include <vtkMRMLNode.h>
#include <vtkCommand.h>
#include <vtkObject.h>
#include <vtkSmartPointer.h>
#include <vtkVariant.h>

class vtkIntString : public vtkObject
{
public:
  vtkTypeMacro(vtkIntString, vtkObject);
  static vtkIntString* New();

  vtkGetMacro(Result, int); 
  vtkSetMacro(Result, int);
  vtkSetMacro(Message, std::string);
  vtkGetMacro(Message, std::string);

protected:

  int Result;
  std::string Message;
  vtkIntString();
  ~vtkIntString() override;

private:

  vtkIntString(const vtkIntString&) = delete;
  void operator=(const vtkIntString&) = delete;
};

class vtkBoolString : public vtkObject
{
public:
  vtkTypeMacro(vtkBoolString, vtkObject);
  static vtkBoolString* New();

  vtkGetMacro(Result, bool); 
  vtkSetMacro(Result, bool);
  vtkSetMacro(Message, std::string);
  vtkGetMacro(Message, std::string);

protected:
  
    bool Result;
    std::string Message;
    vtkBoolString();
    ~vtkBoolString() override;

private:
  
    vtkBoolString(const vtkBoolString&) = delete;
    void operator=(const vtkBoolString&) = delete;
  };

#endif  // vtkCustomTypes_h