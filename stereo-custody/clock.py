"""Desktop-only corroboration of port reports at the application's 20 ms clock."""
from collections import deque

RECEIVE_SHA = "6389d766ad3d4d65b885bccb615a23807cadd8ef4f14d846214cea8a4dd96f73"


class CustodyClock:
    def __init__(self):
        self.captures=deque()
        self.sequence=-1
        self.last_capture=-1
        self.last_result="awaiting-frame"
        self.previous_tick=None
        self.in_band_since=None

    def tick(self,now,port_in_band,packets,*,calibrated,stationary,epoch):
        if self.previous_tick is not None and now != self.previous_tick+20:
            self.captures.clear()
            self.in_band_since=None
        self.previous_tick=now
        if not port_in_band or not stationary or not calibrated:
            self.in_band_since=None
        elif self.in_band_since is None:
            self.in_band_since=now
        if not calibrated or not stationary:
            self.captures.clear()
            self.last_result="uncalibrated-or-moving"
            return False
        for packet in packets:
            capture=packet["capture_ms"]
            valid=(packet["pose_sha256"]==RECEIVE_SHA and packet["epoch"]==epoch and packet["sequence"]>self.sequence and
                   self.last_capture<capture<=packet["arrived_ms"]<=now and now-capture<=40)
            if not valid:
                self.captures.clear()
                self.last_result="invalid-frame-clock-or-identity"
                return False
            if self.last_capture>=0 and capture-self.last_capture>21:
                self.captures.clear()
            self.sequence,self.last_capture=packet["sequence"],capture
            if (packet["observation"]["status"]=="position-witness" and self.in_band_since is not None
                    and capture>=self.in_band_since):
                self.captures.append(capture)
                self.last_result="corroborating"
            else:
                self.captures.clear()
                self.last_result=packet["observation"]["status"]
        if not port_in_band or now-self.last_capture>40:
            self.captures.clear()
        while self.captures and self.captures[0]<now-160:
            self.captures.popleft()
        if len(self.captures)>=5 and self.captures[-1]-self.captures[0]>=68:
            self.last_result="independent-position-corroborated"
            return True
        return False
