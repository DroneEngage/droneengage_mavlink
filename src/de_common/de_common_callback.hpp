#ifndef DE_CALLBACK_HPP
#define DE_CALLBACK_HPP


namespace de
{
namespace comm
{
class CCommon_Callback
{
public:
    

    virtual void OnConnectionStatusChangedWithAndruavServer (const int status) {};
};

}
}
#endif