package main

import "log"
import "time"
import "fmt"
import "container/ring"

type LocationStatus int
type SleepStatus int

const (
	LocationIdle LocationStatus = iota
	LocationEnter
	LocationStay
)

const (
	SleepIdle SleepStatus = iota
	SleepEnter
	Sleeping
)

type Location struct {
	LocStatus    LocationStatus
	LastModified time.Time
	NearStation  string
}

type Sleep struct {
	SlpStatus     SleepStatus
	LastInactived time.Time
}

type AccelerometerElement struct {
	AccelerometerSensorValue
	Modified time.Time
}

type Accelerometer struct {
	RingBuf     *ring.Ring
	LastTweeted time.Time
	AvgAcc      int
}

type CatStatus struct {
	TwitterStatusUploader
	loc               Location
	sleep             Sleep
	Acc               Accelerometer
	LastTweetedPoopHi time.Time
}

func (l *Location) Update(uploader StatusUploader, msg *Message, config *Configuration) {
	switch l.LocStatus {
	case LocationIdle:
		// transit if the cat is near some station
		if msg.CatLq > config.Settings["NearSomewhereThreshold"] {
			l.LocStatus = LocationEnter
			l.LastModified = time.Now()
			l.NearStation = msg.StationAddr
		}
	case LocationEnter:
		if msg.StationAddr == l.NearStation {
			if msg.CatLq > config.Settings["NearSomewhereThreshold"] {
				duration := time.Since(l.LastModified)
				if duration.Seconds() > 30 {
					l.LocStatus = LocationStay
					l.LastModified = time.Now()
					stype := config.GetStationType(msg.StationAddr)
					if stype == StationToiletType {
						status := fmt.Sprintf("%s (%s)", config.Tweets["Toilet"], time.Now())
						uploader.Upload(status)
					} else if stype == StationDrinkType {
						status := fmt.Sprintf("%s (%s)", config.Tweets["Drinking"], time.Now())
						uploader.Upload(status)
					} else if stype == StationMealType {
						status := fmt.Sprintf("%s (%s)", config.Tweets["Hungry"], time.Now())
						uploader.Upload(status)
					}
				}
			} else {
				l.LocStatus = LocationIdle
			}
		}
	case LocationStay:
		if msg.StationAddr == l.NearStation && msg.CatLq <= config.Settings["NearSomewhereThreshold"] {
			l.LocStatus = LocationIdle
			l.LastModified = time.Now()
		}
	default:
		// unreached
		break

	}
	// unreached
	return
}

func (s *Sleep) Update(uploader StatusUploader, msg *Message, config *Configuration) {
	switch s.SlpStatus {
	case SleepIdle:
		if msg.Type == InactivePacket {
			s.SlpStatus = SleepEnter
			s.LastInactived = time.Now()
		}
	case SleepEnter:
		if msg.Type == BeaconPacket {
			duration := time.Since(s.LastInactived)
			if uint(duration.Seconds()) > config.Settings["SleepThreshold"] {
				s.SlpStatus = Sleeping
				status := fmt.Sprintf("%s (%s)", config.Tweets["Sleeping"], time.Now())
				uploader.Upload(status)
			}
		} else {
			s.SlpStatus = SleepIdle
		}
	case Sleeping:
		if msg.Type != BeaconPacket {
			if uint(msg.X+msg.Y+msg.Z) > config.Settings["ActiveThresholdWhenSleeping"] {
				s.SlpStatus = SleepIdle
			}
		}
	default:
		// unreached
		break

	}
	// unreached
	return
}

func (a *Accelerometer) Update(uploader StatusUploader, msg *Message, config *Configuration) {
	if a.RingBuf == nil {
		// init
		a.RingBuf = ring.New(20)
	}
	// update ring buf
	elem := AccelerometerElement{AccelerometerSensorValue: msg.AccelerometerSensorValue,
		Modified: time.Now(),
	}
	a.RingBuf.Value = elem
	// must execute moving next
	next := a.RingBuf.Next()
	defer func() { a.RingBuf = next }()

	// analyze
	var SumAcc int
	var NumElem int
	f := func(val interface{}) {
		if val != nil {
			elem := val.(AccelerometerElement)
			SumAcc += elem.X
			SumAcc += elem.Y
			SumAcc += elem.Z
			NumElem++
		}
	}
	a.RingBuf.Do(f)
	AvgAcc := SumAcc / NumElem
	if NumElem > 10 {
		a.AvgAcc = AvgAcc
	}
	log.Printf("[acc] num %v, sum %v, avg %v", NumElem, SumAcc, AvgAcc)
	duration := time.Since(a.LastTweeted)
	if NumElem > 10 && uint(AvgAcc) > config.Settings["ActiveThreshold"] && duration.Seconds() > 60 {
		status := fmt.Sprintf("%s (%s)", config.Tweets["Playing"], time.Now())
		uploader.Upload(status)
		a.LastTweeted = time.Now()
	}
}

func (a *Accelerometer) IsActive(config *Configuration) bool {
	return uint(a.AvgAcc) > config.Settings["ActiveThreshold"]
}

func (cs *CatStatus) Update(msg *Message, config *Configuration) {
	cs.loc.Update(cs, msg, config)
	cs.sleep.Update(cs, msg, config)
	cs.Acc.Update(cs, msg, config)
	stype := config.GetStationType(cs.loc.NearStation)
	duration := time.Since(cs.loc.LastModified)
	if (stype == StationToiletType) && (duration.Seconds() < 60) && cs.Acc.IsActive(config) {
		durationLastPoop := time.Since(cs.LastTweetedPoopHi)
		if durationLastPoop.Seconds() > 60 {
			status := fmt.Sprintf("%s (%s)", config.Tweets["PoopHi"], time.Now())
			cs.Upload(status)
			cs.LastTweetedPoopHi = time.Now()
		}
	}
}
