package main

import (
	"bytes"
	"github.com/tarm/goserial"
	"io"
	"log"
	"strconv"
	"strings"
)

type SerialMessageReceiver struct {
	rwc io.ReadWriteCloser
	buf []byte
}

func NewSerialMessageReceiver(name string, baud int) MessageReceiver {
	c := &serial.Config{Name: name, Baud: baud}
	s, err := serial.OpenPort(c)
	if err != nil {
		log.Fatal(err)
	}
	return &SerialMessageReceiver{rwc: s, buf: make([]byte, 128)}
}

func (smr *SerialMessageReceiver) Recv() *Message {
	smr.buf = smr.buf[:0]
	buf := make([]byte, 128)
	for true {
		n, err := smr.rwc.Read(buf)
		if err != nil {
			log.Fatal(err)
		}
		smr.buf = append(smr.buf, buf[:n]...)
		i := bytes.IndexByte(smr.buf, ']')
		if i != -1 {
			// received a message
			msgbuf := smr.buf[:i]
			log.Printf("[msg] %s", msgbuf)
			// skip ]
			i++
			// left buffer
			smr.buf = smr.buf[i:]
			return bytes2msg(msgbuf)
		}
	}
	// unreached
	return nil
}

func bytes2msg(buf []byte) *Message {
	if len(buf) < 6 {
		log.Printf("[err] message length too small %d", len(buf))
		return nil
	}
	// trim leading "[PKT "
	trimmed := buf[5:]
	cols := strings.Split(string(trimmed), ",")
	if len(cols) != 11 {
		log.Printf("[err] invalid number of cols %d", len(cols))
		return nil
	}
	var msg Message
	for i, col := range cols {
		vals := strings.Split(col, ":")
		if len(vals) != 2 {
			log.Printf("[err] invalid value format %s", col)
			return nil
		}
		switch i {
		case 0:
			msg.StationAddr = vals[1]
		case 1:
			plen, err := strconv.ParseInt(vals[1], 10, 16)
			if err != nil {
				log.Printf("[err] invalid PayloadLen %s", vals[1])
				return nil
			}
			msg.PayloadLen = uint(plen)
		case 2:
			seq, err := strconv.ParseInt(vals[1], 10, 16)
			if err != nil {
				log.Printf("[err] invalid SeqNum %s", vals[1])
				return nil
			}
			msg.SeqNum = uint(seq)
		case 3:
			lq, err := strconv.ParseInt(vals[1], 10, 16)
			if err != nil {
				log.Printf("[err] invalid StationLq %s", vals[1])
				return nil
			}
			msg.StationLq = uint(lq)
		case 4:
			ts, err := strconv.ParseInt(vals[1], 10, 32)
			if err != nil {
				log.Printf("[err] invalid Timestamp %s", vals[1])
				return nil
			}
			msg.Timestamp = uint(ts)
		case 5:
			ptype := String2PacketType(vals[1])
			if ptype == InvalidPacket {
				log.Printf("[err] invalid PacketType %s", vals[1])
				return nil
			}
			msg.Type = ptype
		case 6:
			x, err := strconv.ParseInt(vals[1], 10, 16)
			if err != nil {
				log.Printf("[err] invalid X %s", vals[1])
				return nil
			}
			msg.X = int(x)
		case 7:
			y, err := strconv.ParseInt(vals[1], 10, 16)
			if err != nil {
				log.Printf("[err] invalid Y %s", vals[1])
				return nil
			}
			msg.Y = int(y)
		case 8:
			z, err := strconv.ParseInt(vals[1], 10, 16)
			if err != nil {
				log.Printf("[err] invalid Z %s", vals[1])
				return nil
			}
			msg.Z = int(z)
		case 9:
			msg.CatAddr = vals[1]
		case 10:
			lq, err := strconv.ParseInt(vals[1], 10, 16)
			if err != nil {
				log.Printf("[err] invalid CatLq %s", vals[1])
				return nil
			}
			msg.CatLq = uint(lq)
		}
	}
	return &msg
}
