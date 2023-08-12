# -*- coding: utf-8 -*-
'''
适用于NEC编码的红外信号解码器
'''

#import libraries#============================
import time
from micropython import schedule
from machine import Pin,Timer

#consts#======================================
START_OF_FRAME_LOW_us:int = int(9 * 1000)
START_OF_FRAME_HEIGHT_us:int = int(4.5 * 1000)

PULS_LOW_us:int = int(0.560 * 1000)
PULS_HIGHT_0_us:int = int(0.560 * 1000)
PULS_HIGHT_1_us:int = int(1.680 * 1000)

TIME_TOLERANCE_us:int = int(0.5 * 1000)
TIMEOUT_TIME_us = 100 * 1000


def _time_update(func:function)->function:
    def ret(self:NEC):
        self._time_last = self._time_now
        self._time_now = time.ticks_us()
        self._time_delta = self._time_now - self._time_last
        func(self)
    return ret

def _convert_list_to_int(ls:list)->int:
    ret = 0
    for b in ls:
        ret <<= 1
        if b:
            ret += 1
    return ret

def _convert_int_to_list(i:int, length:int = -1)->list:
    ret = []
    while( i != 0 and i > 0 if length < 0 else len(ret) < length ):
        ret.append(i & 1)
        i >>= 1
    ret.reverse()
    return ret

def _data_check(a:int, b:int, length:int)->bool:
    return (1<<length)-1 == a ^ b

def _data_check_and_send(self:NEC)->None:
    user:int = _convert_list_to_int(self._bits[:USER_BIT_COUNT])
    user_reverted:int = _convert_list_to_int(self._bits[USER_BIT_COUNT:USER_BIT_COUNT+USER_REVERTED_BIT_COUNT])
    data:int = _convert_list_to_int(self._bits[-DATA_REVERTED_BIT_COUNT-DATA_BIT_COUNT:-DATA_REVERTED_BIT_COUNT])
    data_reverted:int = _convert_list_to_int(self._bits[-DATA_REVERTED_BIT_COUNT:])
    if _data_check(user, user_reverted, USER_BIT_COUNT) and _data_check(data, data_reverted, DATA_BIT_COUNT):
        print(f"User:{user} Data:{data}")
        if self._on_receive_data:
            self._on_receive_data(user, data)
    else:
        print("NEC Receive Abort: DATA_BIT_HANDLER_HEIGHT_end")

@_time_update
def START_OF_FRAME_LOW(self:NEC)->None:
    if self._pin_in.value() == 1:
        print("NEC Receive Abort: START_OF_FRAME_LOW")
        self._init_receiver_state()
    else:
        schedule(NEC._start_receive_data_watchdog, self)
        self._on_pin_in_interrupt_handler = START_OF_FRAME_HEIGHT

@_time_update
def START_OF_FRAME_HEIGHT(self:NEC)->None:
    if abs(self._time_delta - START_OF_FRAME_LOW_us) > TIME_TOLERANCE_us:
        print("NEC Receive Abort: START_OF_FRAME_HEIGHT")
        self._init_receiver_state()
    else:
        self._on_pin_in_interrupt_handler = PULS_LOW

@_time_update
def PULS_LOW(self:NEC)->None:
    self._data_bit_handler_low(self)

def DATA_BIT_HANDLER_LOW_first(self:NEC)->None:
    if abs(self._time_delta - START_OF_FRAME_HEIGHT_us) > TIME_TOLERANCE_us:
        print("NEC Receive Abort: DATA_BIT_HANDLER_LOW_first")
        self._init_receiver_state()
    else:
        self._on_pin_in_interrupt_handler = PULS_HEIGHT
        self._data_bit_handler_low = DATA_BIT_HANDLER_LOW
        
def DATA_BIT_HANDLER_LOW(self:NEC)->None:
    if abs(self._time_delta - PULS_HIGHT_0_us) < TIME_TOLERANCE_us:
        self._bits[self._bits_ptr] = False
    elif abs(self._time_delta - PULS_HIGHT_1_us) < TIME_TOLERANCE_us:
        self._bits[self._bits_ptr] = True
    else:
        print("NEC Receive Abort: DATA_BIT_HANDLER_LOW")
        self._init_receiver_state()
    self._bits_ptr += 1
    self._on_pin_in_interrupt_handler = PULS_HEIGHT
    if self._bits_ptr == TOTAL_BIT_COUNT:
        self._data_bit_handler_height = DATA_BIT_HANDLER_HEIGHT_end

@_time_update
def PULS_HEIGHT(self:NEC)->None:
    self._data_bit_handler_height(self)

def DATA_BIT_HANDLER_HEIGHT(self:NEC)->None:
    if abs(self._time_delta - PULS_LOW_us) > TIME_TOLERANCE_us:
        print("NEC Receive Abort: DATA_BIT_HANDLER_HEIGHT")
        self._init_receiver_state()
    else:
        self._on_pin_in_interrupt_handler = PULS_LOW

def DATA_BIT_HANDLER_HEIGHT_end(self:NEC)->None:
    schedule(_data_check_and_send, self)
    self._init_receiver_state()


USER_BIT_COUNT:int = 8
USER_REVERTED_BIT_COUNT:int = 8
DATA_BIT_COUNT:int = 8
DATA_REVERTED_BIT_COUNT:int = 8

TOTAL_BIT_COUNT:int = USER_BIT_COUNT + USER_REVERTED_BIT_COUNT + DATA_BIT_COUNT + DATA_REVERTED_BIT_COUNT

#class#=======================================
class NEC:

    def __init__(self, pin_in:Pin = None, pin_out:Pin = Pin(25), timer:Timer = Timer(-1), on_receive_data:function = None)->None:
        '''
        初始化一个NEC编码红外接收器
        
        params：
            pin_in：使用的Pin引脚作为接受端
            pin_out：使用的Pin引脚作为输入端
            timer: 使用的计时器
            on_receive_data： 接收数据后的回调
        '''
        self._pin_in:Pin = pin_in
        self._pin_out:Pin = pin_out
        self._timer:Timer = timer
        self._on_receive_data:function = on_receive_data
        
        self.start_receive_data()
    
    def start_receive_data(self)->None:
        '''
        启用红外接收
        '''
        self._bits:list = [False for i in range(TOTAL_BIT_COUNT)]
        if self._pin_in is None:
            print("启动红外接收失败，因为没有配置接收引脚")
            return
        self._init_receiver_state()
        self._pin_in.init(mode = Pin.IN, pull = None)
        self._pin_in.irq(handler = self._on_pin_in_interrupt , trigger = Pin.IRQ_FALLING | Pin.IRQ_RISING, hard = True)
        print("红外接收已开启")
    
    def stop_receive_data(self)->None:
        if self._pin_in is not None:
            self._pin_in.irq(handler = None , trigger = 0, hard = True)
        print("红外接收已关闭")
        
    def send_data(self, user:int, data:int)->None:
        self._pin_out.off()
        self._pin_out.init(mode = Pin.OUT, pull = None)
        user_revert:int = user ^ (1<<USER_BIT_COUNT)-1
        data_revert:int = data ^ (1<<DATA_BIT_COUNT)-1
        signal:list = [0,START_OF_FRAME_LOW_us,START_OF_FRAME_HEIGHT_us]
        def bit_to_signal(ls:list)->list:
            ret = []
            for bit in ls:
                ret.append(PULS_LOW_us)
                ret.append(PULS_HIGHT_1_us if bit else PULS_HIGHT_0_us)
            return ret
        signal.extend(bit_to_signal(_convert_int_to_list(user, USER_BIT_COUNT)))
        signal.extend(bit_to_signal(_convert_int_to_list(user_revert, USER_BIT_COUNT)))
        signal.extend(bit_to_signal(_convert_int_to_list(data, DATA_BIT_COUNT)))
        signal.extend(bit_to_signal(_convert_int_to_list(data_revert, DATA_BIT_COUNT)))
        signal.append(PULS_LOW_us)
        for i in range(len(signal)-1):
            signal[i+1] += signal[i]
        fire_time = time.ticks_us() + TIMEOUT_TIME_us
        for t in signal:
            while(time.ticks_us() < t+fire_time):
                pass
            self._pin_out.toggle()
        self._pin_out.init(mode = Pin.IN, pull = None)
    
    def _on_pin_in_interrupt(self, args:Pin)->None:
        '''
        状态机式 中断处理函数
        '''
        self._on_pin_in_interrupt_handler(self)
    
    def _init_receiver_state(self)->None:
        '''
        接收状态机 初始化
        '''
        self._time_now = time.ticks_us()
        self._time_last = self._time_now
        self._time_delta = 0
        
        self._bits_ptr:int = 0
        
        self._on_pin_in_interrupt_handler:function = START_OF_FRAME_LOW
        self._data_bit_handler_low:function = DATA_BIT_HANDLER_LOW_first
        self._data_bit_handler_height:function = DATA_BIT_HANDLER_HEIGHT
    
    def _start_receive_data_watchdog(self)->None:
        '''
        数据接收 看门狗
        '''
        def on_receive_data_timeout(timer:Timer)->None:
            if self._on_pin_in_interrupt_handler != START_OF_FRAME_LOW: 
                print("NEC Receive Abort: Receive Timeout")
                self._init_receiver_state()
        self._timer.init(mode = Timer.ONE_SHOT, period = int(TIMEOUT_TIME_us/1000), callback = on_receive_data_timeout)
#=============================================
