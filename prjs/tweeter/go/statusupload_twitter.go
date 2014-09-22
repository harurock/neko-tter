package main

import "github.com/ChimeraCoder/anaconda"
import "log"

type TwitterStatusUploader struct {
	Addr              string
	ApiKey            string
	ApiSecret         string
	AccessToken       string
	AccessTokenSecret string
	api               *anaconda.TwitterApi
}

func (tsu *TwitterStatusUploader) Auth() {
	tsu.api = anaconda.NewTwitterApi(tsu.AccessToken, tsu.AccessTokenSecret)
}

func (tsu *TwitterStatusUploader) Upload(status string) error {
	anaconda.SetConsumerKey(tsu.ApiKey)
	anaconda.SetConsumerSecret(tsu.ApiSecret)
	_, err := tsu.api.PostTweet(status, nil)
	if err != nil {
		log.Printf("An error occured while uploading. err:%v", err)
	}
	return err
}
