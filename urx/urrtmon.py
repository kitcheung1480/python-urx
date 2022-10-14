'''
Module for implementing a UR controller real-time monitor over socket port 30003.
Confer http://support.universal-robots.com/Technical/RealTimeClientInterface
Note: The packet lenght given in the web-page is 740. What is actually received from the controller is 692. It is assumed that the motor currents, the last group of 48 bytes, are not send.
Originally Written by Morten Lind
'''
import logging
import socket
import struct
import time
import threading
from copy import deepcopy

import numpy as np

import math3d as m3d

__author__ = "Morten Lind, Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011, NTNU/SINTEF Raufoss Manufacturing AS"
__credits__ = ["Morten Lind, Olivier Roulet-Dubonnet"]
__license__ = "LGPLv3"

class URRTMonitor(threading.Thread):

    # Struct for revision of the UR controller giving 692 bytes
    rtstruct692 = struct.Struct('>d6d6d6d6d6d6d6d6d18d6d6d6dQ')

    # for revision of the UR controller giving 540 byte. Here TCP
    # pose is not included!
    rtstruct540 = struct.Struct('>d6d6d6d6d6d6d6d6d18d')

    def __init__(self, urHost):
        threading.Thread.__init__(self)
        self.logger = logging.getLogger(self.__class__.__name__)
        self.daemon = True
        self._stop_event = True
        self._dataEvent = threading.Condition()
        self._dataAccess = threading.Lock()
        self._rtSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._rtSock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._urHost = urHost
        # Package data variables
        self._timestamp = None
        self._ctrlTimestamp = None
        self._qActual = None
        self._qTarget = None
        self._tcp = None
        self._tcp_force = None
        self.__recvTime = 0
        self._last_ctrl_ts = 0
        # self._last_ts = 0
        self._buffering = False
        self._buffer_lock = threading.Lock()
        self._buffer = []
        self._csys = None
        self._csys_lock = threading.Lock()

        self.start()
        self.wait()  # make sure we got some data before someone calls us

    def set_csys(self, csys):
        with self._csys_lock:
            self._csys = csys

    def __recv_bytes(self, nBytes):
        ''' Facility method for receiving exactly "nBytes" bytes from
        the robot connector socket.'''
        # Record the time of arrival of the first of the stream block
        recvTime = 0
        pkg = b''
        while len(pkg) < nBytes:
            pkg += self._rtSock.recv(nBytes - len(pkg))
            if recvTime == 0:
                recvTime = time.time()
        self.__recvTime = recvTime
        return pkg

    def wait(self, timeout=0.05):
        with self._dataEvent:
            self._dataEvent.wait(timeout)

    def q_actual(self, wait=False, timestamp=False):
        """ Get the actual joint position vector."""
        if wait:
            self.wait()
        with self._dataAccess:
            if timestamp:
                return self._timestamp, self._qActual
            else:
                return self._qActual
    getActual = q_actual

    def qd_actual(self, wait=False, timestamp=False):
        """ Get the actual joint velocity vector."""
        if wait:
            self.wait()
        with self._dataAccess:
            if timestamp:
                return self._timestamp, self._qdActual
            else:
                return self._qdActual

    def q_target(self, wait=False, timestamp=False):
        """ Get the target joint position vector."""
        if wait:
            self.wait()
        with self._dataAccess:
            if timestamp:
                return self._timestamp, self._qTarget
            else:
                return self._qTarget

    def tcp_pose(self, wait=False, timestamp=False, ctrlTimestamp=False):
        """ Return the tool pose values."""
        if wait:
            self.wait()
        with self._dataAccess:
            tcp = self._tcp
            if ctrlTimestamp or timestamp:
                ret = [tcp]
                if timestamp:
                    ret.insert(-1, self._timestamp)
                if ctrlTimestamp:
                    ret.insert(-1, self._ctrlTimestamp)
                return ret
            else:
                return tcp

    def tcp_force(self, wait=False, timestamp=False):
        """ Get the tool force. The returned tool force is a
        six-vector of three forces and three moments."""
        if wait:
            self.wait()
        with self._dataAccess:
            # tcp = self._fwkin(self._qActual)
            tcp_force = self._tcp_force
            if timestamp:
                return self._timestamp, tcp_force
            else:
                return tcp_force

    def robot_mode(self, wait=False):
        if wait:
            self.wait()
        with self._dataEvent:
            return self._robotMode
    
    def is_running(self, wait=False):
        robot_mode = self.robot_mode(wait)
        print("robot_mode", robot_mode)
        return robot_mode == 7

    def program_state(self, wait=False):
        if wait:
            self.wait()
        with self._dataEvent:
            print("program state", self._state)
            return self._state
    
    def digital_input(self, index = None, wait=False):
        if wait:
            self.wait()
        with self._dataEvent:
            digit_in = self._digit_in
            if index is None:
                bits = []
                for i in range(8):
                    bits.append(digit_in >> i & 1)
                return bits
            else:
                bit = digit_in >> index & 1
                return bit

    def digital_output(self, index = None, wait=False):
        if wait:
            self.wait()
        with self._dataEvent:
            digit_out = self._digit_out
            if index is None:
                bits = []
                for i in range(8):
                    bits.append(digit_out >> i & 1)
                return bits
            else:
                bit = digit_out >> index & 1
                return bit


    # ============================================================================================
    # custome send program in rtmon
    def send_program(self, prog):
        """
        send program to robot in URRobot format
        If another program is send while a program is running the first program is aborded.
        """
        prog.strip()
        # self.logger.debug("Enqueueing program: %s", prog)
        if not isinstance(prog, bytes):
            prog = prog.encode()

        self._rtSock.send(prog + b"\n")
    # ============================================================================================

    def __recv_rt_data(self):
        head = self.__recv_bytes(4)
        # Record the timestamp for this logical package
        timestamp = self.__recvTime
        pkgsize = struct.unpack('>i', head)[0]
        self.logger.debug(
            'Received header telling that package is %s bytes long',
            pkgsize)
        payload = self.__recv_bytes(pkgsize - 4)
        if pkgsize >= 692:
            unp = self.rtstruct692.unpack(payload[:self.rtstruct692.size])
        elif pkgsize >= 540:
            unp = self.rtstruct540.unpack(payload[:self.rtstruct540.size])
        else:
            self.logger.warning(
                'Error, Received packet of length smaller than 540: %s ',
                pkgsize)
            return

        with self._dataAccess:
            self._timestamp = timestamp
            # it seems that packet often arrives packed as two... maybe TCP_NODELAY is not set on UR controller??
            # if (self._timestamp - self._last_ts) > 0.010:
            # self.logger.warning("Error the we did not receive a packet for {}s ".format( self._timestamp - self._last_ts))
            # self._last_ts = self._timestamp
            self._ctrlTimestamp = np.array(unp[0])
            if self._last_ctrl_ts != 0 and (
                    self._ctrlTimestamp -
                    self._last_ctrl_ts) > 0.010:
                self.logger.warning(
                    "Error the controller failed to send us a packet: time since last packet %s s ",
                    self._ctrlTimestamp - self._last_ctrl_ts)
            self._last_ctrl_ts = self._ctrlTimestamp
            """
            self._qActual = np.array(unp[31:37])
            self._qdActual = np.array(unp[37:43])
            self._qTarget = np.array(unp[1:7])
            self._tcp_force = np.array(unp[67:73])
            self._tcp = np.array(unp[73:79])
            """
            # print(len(unp))
            self._qTarget = np.array(unp[1:7])
            # self._qdTarget = np.array(unp[7:13])
            # self._qddTarget = np.array(unp[13:19])
            # self._I_Target = np.array(unp[19:25])
            # self._M_Target = np.array(unp[25:31])
            self._qActual = np.array(unp[31:37])
            self._qdActual = np.array(unp[37:43])
            # self._IActual = np.array(unp[43:49])
            # self._IControl = np.array(unp[49:55])
            self._tcp = np.array(unp[55:61]) # _tcpActual
            self._tcp_speed = np.array(unp[61:67])
            self._tcp_force = np.array(unp[67:73])
            # self._tcpTarget = np.array(unp[73:79])
            # self._tcpSpeedTarget = np.array(unp[79:85])
            self._digit_in = int(unp[85])
            # self._motorTemperatures = np.array(unp[86:92])
            # # self._controllerTimer = unp[92]
            # self._robotMode = int(unp[94]) # Confirm Safety: 1, Running: 7
            # self._accelerometer = np.array(unp[108:111])
            # self._speedScaling = float(unp[117])
            # self._momentum = float(unp[118])
            # self._digit_out = int(unp[130])
            # self._state = int(unp[131])
            # self._elbowPosition = np.array(unp[132:135])
            # self._elbowVelocity = np.array(unp[135:138])

            if self._csys:
                with self._csys_lock:
                    # might be a godd idea to remove dependancy on m3d
                    tcp = self._csys.inverse * m3d.Transform(self._tcp)
                self._tcp = tcp.pose_vector
        if self._buffering:
            with self._buffer_lock:
                self._buffer.append(
                    (self._timestamp,
                     self._ctrlTimestamp,
                     self._tcp,
                     self._qActual))  # FIXME use named arrays of allow to configure what data to buffer

        with self._dataEvent:
            self._dataEvent.notifyAll()

    def start_buffering(self):
        """
        start buffering all data from controller
        """
        self._buffer = []
        self._buffering = True

    def stop_buffering(self):
        self._buffering = False

    def try_pop_buffer(self):
        """
        return oldest value in buffer
        """
        with self._buffer_lock:
            if len(self._buffer) > 0:
                return self._buffer.pop(0)
            else:
                return None

    def pop_buffer(self):
        """
        return oldest value in buffer
        """
        while True:
            with self._buffer_lock:
                if len(self._buffer) > 0:
                    return self._buffer.pop(0)
            time.sleep(0.001)

    def get_buffer(self):
        """
        return a copy of the entire buffer
        """
        with self._buffer_lock:
            return deepcopy(self._buffer)

    def get_all_data(self, wait=True):
        """
        return all data parsed from robot as a dict
        """
        if wait:
            self.wait()
        with self._dataAccess:
            return dict(
                timestamp=self._timestamp,
                ctrltimestamp=self._ctrlTimestamp,
                qActual=self._qActual,
                qTarget=self._qTarget,
                tcp=self._tcp,
                tcp_force=self._tcp_force
            )

    def stop(self):
        # print(self.__class__.__name__+': Stopping')
        self._stop_event = True

    def close(self):
        self.stop()
        try:
            self.join()
        except RuntimeError:
            pass
        except Exception as ex:
            raise(ex)
        with self._dataEvent: #wake up any thread that may be waiting for data before we close. Should we do that?
            self._dataEvent.notifyAll()

    def run(self):
        self._stop_event = False
        try:
            self._rtSock.connect((self._urHost, 30003))
            while not self._stop_event:
                self.__recv_rt_data()
            self._rtSock.close()
        except OSError as ex:
            # print("RT", ex)
            pass
