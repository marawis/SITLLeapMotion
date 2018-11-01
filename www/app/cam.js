$(function() {
   $("#cam").networkCamera({
        url: 'https://www.google.com/logos/doodles/2015/fifa-women-world-cup-winner-tbd-country-1-5173664725073920.3-hp.jpg',
        streaming: true
      });

    $('#cam').networkCamera('stream');
});