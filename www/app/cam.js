$(function() {
   $("#cam").networkCamera({
        url: 'http://localhost:5050/video_feed',
        streaming: true
      });

    //$('#cam').networkCamera('stream');
});