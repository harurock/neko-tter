package main

import (
	"strconv"
	//"fmt"
	"log"
)

type PacketType int

const (
	BeaconPacket PacketType = iota
	ActivePacket
	InactivePacket
	InvalidPacket
)

type AccelerometerSensorValue struct {
	X int
	Y int
	Z int
}

type Message struct {
	StationAddr string
	PayloadLen  uint
	SeqNum      uint
	StationLq   uint
	Timestamp   uint
	Type        PacketType
	AccelerometerSensorValue
	CatAddr string
	CatLq   uint
}

type MessageReceiver interface {
	Recv() *Message
}

func String2PacketType(src string) PacketType {
	i, err := strconv.ParseInt(src, 16, 16)
	if err != nil {
		log.Printf("[err] ParseInt failed %v", err)
		return InvalidPacket
	}
	ptype := i & 0x7f
	if ptype > int64(InactivePacket) {
		return InvalidPacket
	}
	return PacketType(ptype)
}
