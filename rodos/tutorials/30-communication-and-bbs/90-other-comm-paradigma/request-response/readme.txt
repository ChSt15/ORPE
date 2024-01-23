
Non Reliable Synchronous Request (client) / Response (Server)



author Sergio Montenegro
date 2022/05/14 




The class ReqResp (reqresp.h) is an extension from topic. For each
ReqResp object there may be only one server. If there are more than one client
they shall not send request the the same time. The best is: Only one
client and only one server per ReqResp object.

The same Topic on which ReqResp is based is used to transmit both: 
the request message and the response message.

The "client" publishes the request message and waits until it gets the answer/response from the server.
It is possible to set a time out to continue even if the answer did not come.


