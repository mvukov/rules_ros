## Releasing

Releases are automated on a cron trigger.
If you do nothing, eventually the newest commits will be released automatically. See .github/workflows/tag.yaml.

To trigger a release where the new version can be determined automatically from the commit history, just navigate to
https://github.com/mvukov/rules_ros/actions/workflows/tag.yaml
and press the "Run workflow" button.

If you need control over the next release version, for example when making a release candidate for a new major,
then: tag the repo and push the tag, for example

```sh
% git fetch
% git tag v1.0.0-rc0 origin/main
% git push origin v1.0.0-rc0
```

Then watch the automation run on GitHub actions which creates the release.
