const char body[] PROGMEM = R"===(
  <!DOCTYPE html>
    <html>
    <body>
    <!--TODO some ui feedback for button presses or something-->
    <span>
      <button onclick="window.location.href = 'switchmode?val=1';">Wall Follow</button>
      <button onclick="window.location.href = 'switchmode?val=2';">Push Police Car</button>
      <button onclick="window.location.href = 'switchmode?val=3';">Nav Fake Trophy</button>
      <button onclick="window.location.href = 'switchmode?val=4';">Nav Real Trophy</button>
    </span>
    </body>
    <script>
      document.addEventListener('keydown', function(event) {
        if (!event.repeat) {
          var code = event.code;
          var xhttp = new XMLHttpRequest();
          if (code == 'KeyW' || code == 'KeyS') {
            var str = "straight?val=";
            var res = code == 'KeyW' ? str.concat('0') : str.concat('1');
            xhttp.open("GET", res, true);
          } else if (code == 'KeyA' || code == 'KeyD') {
            var str = "turn?val=";
            var res = code == 'KeyA' ? str.concat('0') : str.concat('1');
            xhttp.open("GET", res, true);
          }
          xhttp.send();
        }
      })
      document.addEventListener('keyup', function(event) {
        var xhttp = new XMLHttpRequest();
        var res = "stop";
        xhttp.open("GET", res, true);
        xhttp.send();
      })
    </script>
    </html>
)===";