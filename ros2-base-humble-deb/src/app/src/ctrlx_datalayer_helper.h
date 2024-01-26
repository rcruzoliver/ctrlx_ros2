/*
 * SPDX-FileCopyrightText: Bosch Rexroth AG
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef CTRLX_DATALAYER_HELPER_H
#define CTRLX_DATALAYER_HELPER_H

 /*! \file
  *
  * This header file provides auxiliary methods to create ctrlX Datalayer client and provider connections to ctrlX CORE devices.
  * It can be used for both running in an app build environment (QEMU VM) and within the snap environment on the ctrlX CORE.
  *
  * Feel free to use it in your projects and to change it if necessary.
  *
  * For ease of use, the default values for IP address, user, password and SSL port are chosen to match the settings of a
  * newly created ctrlX CORE device:
  *
  *    ip="192.168.1.1"
  *    user="boschrexroth"
  *    password="boschrexroth"
  *    ssl_port=443
  *
  * If these values do not suit your use case, explicitly pass the parameters that require different values.
  * Here some examples:
  *
  * 1. ctrlX CORE or ctrlX COREvirtual with another IP address, user and password:
  *
  *     client, client_connection = get_client(system, ip="192.168.1.100", user="admin", password="-$_U/{X$aG}Z3/e<")
  *
  * 2. ctrlX COREvirtual with port forwarding running on the same host as the app build environment (QEMU VM):
  *
  *     client, client_connection = get_client(system, ip="10.0.2.2", ssl_port=8443)
  *
  *     Remarks:
  *     10.0.2.2 is the IP address of the host from the point of view of the app build environment (QEMU VM).
  *     8443 is the host port which is forwarded to the SSL port (=433) of the ctrlX COREvirtual
  *
  *
  * IMPORTANT:
  * You need not change the parameter settings before building a snap and installing the snap on a ctrlX CORE.
  * The method get_connection_string detects the snap environment and uses automatically inter process communication.
  * Therefor the connection string to the ctrlX Datalayer is:
  *
  *     "ipc://"
  *
  */

#include <stdio.h>
#include <iostream>

#include "comm/datalayer/datalayer.h"
#include "comm/datalayer/datalayer_system.h"

  //! Retrieve environment variable SNAP
static const char* snapPath()
{
  return std::getenv("SNAP");
}

//! Test if code is runnning in snap environment
static bool isSnap()
{
  return snapPath() != nullptr;
}

//! Get Datalayer connection string
//! @param[in] ip       IP address of the ctrlX CORE: 10.0.2.2 is ctrlX COREvirtual with port forwarding
//! @param[in] user     User name
//! @param[in] password The password
//! @param[in] sslPort  The port number for SSL: 8443 if ctrlX COREvirtual with port forwarding 8443:443
//! @result Connection string
static std::string getConnectionString(
  const std::string& ip = "192.168.1.1",
  const std::string& user = "boschrexroth",
  const std::string& password = "boschrexroth",
  int sslPort = 443)
{
  if (isSnap())
  {
    std::cout << "INFO Is snap" << std::endl;
    return DL_IPC;
  }

  std::string connectionString = DL_TCP + user + std::string(":") + password + std::string("@") + ip;

  if (443 == sslPort)
  {
    return connectionString;
  }

  return connectionString + std::string("?sslport=") + std::to_string(sslPort);
}

//! Get Datalayer Client instance
//! @param[in] datalayerSystem Datalayer.System instance
//! @param[in] ip       IP address of the ctrlX CORE: 10.0.2.2 is ctrlX COREvirtual with port forwarding
//! @param[in] user     User name
//! @param[in] password The password
//! @param[in] sslPort  The port number for SSL: 8443 if ctrlX COREvirtual with port forwarding 8443:443
//! @result IClient instance or nullptr on error
static comm::datalayer::IClient* getClient(comm::datalayer::DatalayerSystem& datalayerSystem,
                                           
  const std::string& ip = "192.168.1.1",
  const std::string& user = "boschrexroth",
  const std::string& password = "boschrexroth",
  int sslPort = 443)
{
  std::string connectionString = getConnectionString(ip, user, password, sslPort);
  comm::datalayer::IClient* client = datalayerSystem.factory()->createClient(connectionString);
  if (client->isConnected())
  {
    return client;
  }

  delete client;

  return nullptr;
}

//! Get Datalayer Provider instance
//! @param[in] datalayerSystem Datalayer.System instance
//! @param[in] ip       IP address of the ctrlX CORE: 10.0.2.2 is ctrlX COREvirtual with port forwarding
//! @param[in] user     User name
//! @param[in] password The password
//! @param[in] sslPort  The port number for SSL: 8443 if ctrlX COREvirtual with port forwarding 8443:443
//! @result IProvider instance or nullptr on error
static comm::datalayer::IProvider* getProvider(comm::datalayer::DatalayerSystem& datalayerSystem,                                           
  const std::string& ip = "192.168.1.1",
  const std::string& user = "boschrexroth",
  const std::string& password = "boschrexroth",
  int sslPort = 443)
{
  std::string connectionString = getConnectionString(ip, user, password, sslPort);
  comm::datalayer::IProvider* provider = datalayerSystem.factory()->createProvider(connectionString);
  if (provider->start() == DL_OK && provider->isConnected())
  {
    return provider;
  }

  delete provider;

  return nullptr;
}



using comm::datalayer::IProviderNode;

// Basic class Provider node interface for providing data to the system
class MyProviderNode: public IProviderNode
{
private:
  comm::datalayer::Variant m_data;
  bool dynamic=true;
public:
  MyProviderNode(comm::datalayer::Variant data)
    : m_data(data)
  {};
  void setString(const std::string& input)
  { 
    m_data.setValue(input);

  }
    void setStatus(const bool status)
  { 
    m_data.setValue(status);

  }
    void setData(comm::datalayer::Variant data)
  { 
    m_data=data;
  }


   comm::datalayer::Variant getData()
  { 
    return m_data;
  }

    comm::datalayer::Variant* getDataP()
  { 
    return &m_data;
  }
  void setDynamic(const bool myDynamic)
  { 
    this->dynamic=myDynamic;
  }
  bool isTrue(){
    
    if (m_data.getType() == comm::datalayer::VariantType::BOOL8 ){
      const bool mydata= m_data;
      return true&mydata;
    }
    else{
      return false;
    }
  }




  virtual ~MyProviderNode() override {};

  // Create function of an object. Function will be called whenever a object should be created.
  virtual void onCreate(const std::string& address, const comm::datalayer::Variant* data, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    callback(comm::datalayer::DlResult::DL_FAILED, nullptr);
  }

  // Read function of a node. Function will be called whenever a node should be read.
  virtual void onRead(const std::string& address, const comm::datalayer::Variant* data, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    comm::datalayer::Variant dataRead;
    dataRead = m_data;
    callback(comm::datalayer::DlResult::DL_OK, &dataRead);
  }

  // Write function of a node. Function will be called whenever a node should be written.
  virtual void onWrite(const std::string& address, const comm::datalayer::Variant* data, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    std::cout << "INFO onWrite " <<  address << std::endl;
    
    if (data->getType() != m_data.getType() or  this->dynamic!=true )
    {
      callback(comm::datalayer::DlResult::DL_TYPE_MISMATCH, nullptr);
    }

    m_data = *data;
    callback(comm::datalayer::DlResult::DL_OK, data);
  }

  // Remove function for an object. Function will be called whenever a object should be removed.
  virtual void onRemove(const std::string& address, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    callback(comm::datalayer::DlResult::DL_FAILED, nullptr);
  }

  // Browse function of a node. Function will be called to determine children of a node.
  virtual void onBrowse(const std::string& address, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    callback(comm::datalayer::DlResult::DL_FAILED, nullptr);
  }

  // Read function of metadata of an object. Function will be called whenever a node should be written.
  virtual void onMetadata(const std::string& address, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    // Keep this comment! Can be used as sample creating metadata programmatically.
    // callback(comm::datalayer::DlResult::DL_OK, &_metaData);

    // Take metadata from metadata.mddb
    callback(comm::datalayer::DlResult::DL_FAILED, nullptr);
  }
};




#endif

