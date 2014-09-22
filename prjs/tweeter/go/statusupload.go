package main

type StatusUploader interface {
	Upload(status string) error
}
