package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"io/ioutil"
	"log"
	"time"
)

type StationType int

const (
	StationToiletType StationType = iota
	StationMealType
	StationDrinkType
	StationInvalidType
)

type Station struct {
	Addr string
	Type StationType
}

type Configuration struct {
	Cats     []*CatStatus
	Stations []*Station
	Tweets   map[string]string
	Settings map[string]uint
}

func (config *Configuration) GetStationType(Addr string) StationType {
	for _, station := range config.Stations {
		if station.Addr == Addr {
			return station.Type
		}
	}
	return StationInvalidType
}

var (
	deviceName = flag.String("d", "/dev/ttyUSB0", "serial port device(default /dev/ttyUSB0)")
	baudRate   = flag.Int("b", 9600, "serial port speed(default 9600)")
	conf       = flag.String("c", "./tweeter.json", "config file path(default ./tweeter.json)")
)

func init() {
	flag.Parse()
}

func main() {
	log.Printf("device:%q baud:%d", *deviceName, *baudRate)
	smr := NewSerialMessageReceiver(*deviceName, *baudRate)
	buf, err := ioutil.ReadFile(*conf)
	if err != nil {
		log.Fatal(err)
	}
	var config Configuration
	err = json.Unmarshal(buf, &config)
	if err != nil {
		log.Fatal(err)
	}
	for _, cat := range config.Cats {
		cat.Auth()
	}
	LastBattle := time.Now()
	for true {
		msg := smr.Recv()
		if msg != nil {
			cat := findCatbyAddr(config.Cats, msg.CatAddr)
			if cat == nil {
				log.Printf("[ignored] unknown cat address %s", msg.CatAddr)
				continue
			}
			cat.Update(msg, &config)
		} else {
			log.Print("[ignored] invalid message")
		}
		tweetBattle(&LastBattle, &config)
	}
}

func findCatbyAddr(cats []*CatStatus, Addr string) *CatStatus {
	for _, cat := range cats {
		if cat.Addr == Addr {
			return cat
		}
	}
	return nil
}

func tweetBattle(lastBattle *time.Time, config *Configuration) {
	var Num int
	for _, cat := range config.Cats {
		if cat.Acc.IsActive(config) {
			Num++
		}
	}
	if Num > 1 {
		duration := time.Since(*lastBattle)
		if uint(duration.Seconds()) > 60 {
			for _, cat := range config.Cats {
				if cat.Acc.IsActive(config) {
					status := fmt.Sprintf("%s (%s)", config.Tweets["Battle"], time.Now())
					cat.Upload(status)
				}
			}
		}
		*lastBattle = time.Now()
	}
}
