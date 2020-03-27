function showIP()
% SHOWIP Show IP of connected network interfaces
% Stop VM newteorking
% sudo -i
% /usr/bin/vmware-networks --stop
% /usr/bin/vmware-networks --start  
  import('se.hendeby.sensordata.*');
  StreamSensorDataReader.showIPs();
end
