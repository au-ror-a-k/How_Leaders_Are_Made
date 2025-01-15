extensions [matrix]

globals [
  TY N ND m  ; World and simulation parameters
  FSR Stag Exil S_count-ier E_count-ier Idle ; Interaction matrices
  ;if FSR 0 - off 1 - smooth 2 - abrupt
  d perceive dp Di ; Distance and perception matrices
  ;d - current distance | perceive - if perceived |
  ;dp - predicted distance | Di - ideal distance
  d_t-1 perceive_t-1 dp_t-1 Di_t-1
  ;d_t-1 - distance at t-1 | perceive_t-1 - if perceived at t-1 |
  ;dp_t-1 - predicted distance at t-1 | Di_t-1 - ideal distanceat t-1
  d_t+1 dp_t+1
  ;d_t+1 - distance at t+1
  ;dp_t+1 - predicted distance at t+1
  flocking-index ; Flocking index and dissatisfaction
  cyclic change
  cyclic_t-1 change_t-1
  invalid-patch
]

breed [flockers flock]
flockers-own [
  x-coord y-coord heading-radians ; Initial coordinates and heading
  r z; Radius and angle of circular spector
  agent-dissatisfaction ; Individual dissatisfaction
]

to setup
  clear-all
  set invalid-patch patch -100000 -100000
  set N 20  ; Number of agents
  set TY "torus" ; World type: toroidal or bounded

  ifelse (TY = "torus") [
    set m (sqrt (H ^ 2 + W ^ 2) / 2)
  ][
    set m sqrt (H ^ 2 + W ^ 2)
  ]

  resize-world 0 W 0 H
  create-flockers N [
    set x-coord random W
    set y-coord random H
    setxy x-coord y-coord
    set heading-radians random-float 2 * pi
    set heading heading-radians * 180 / pi
    set r radius ; Radius of spectre
    set z circ-spector  ; Angle of circular spector in degrees
    set color red
    set agent-dissatisfaction 0 ; Initialize dissatisfaction
  ]

  set FSR matrix:from-row-list n-values N [n-values N [0]]
  set Stag matrix:from-row-list n-values N [n-values N [P_s]]
  set Exil matrix:from-row-list n-values N [n-values N [P_e]]
  set S_count-ier matrix:from-row-list n-values N [n-values N [0]]
  set E_count-ier matrix:from-row-list n-values N [n-values N [0]]
  set Idle matrix:from-row-list n-values N [n-values N [0]]
  set d matrix:from-row-list n-values N [n-values N [0]]
  set dp matrix:from-row-list n-values N [n-values N[0]]
  set perceive matrix:from-row-list n-values N [n-values N [false]]
  set perceive_t-1 matrix:from-row-list n-values N [n-values N[false]]
  set d_t-1 matrix:from-row-list n-values N [n-values N [0]]
  set dp_t-1 matrix:from-row-list n-values N [n-values N [[0]]]
  set Di matrix:from-row-list n-values N [n-values N [[0]]]
  set d_t+1 matrix:from-row-list n-values N [n-values N [0]]
  set dp_t+1 matrix:from-row-list n-values N [n-values N [[0]]]
  set Di_t-1 matrix:from-row-list n-values N [n-values N [[0]]]
  set change_t-1 matrix:from-row-list n-values N [n-values N [[0]]]
  set cyclic_t-1 matrix:from-row-list n-values N [n-values N [[0]]]
  set change matrix:from-row-list n-values N [n-values N [[0]]]
  set cyclic matrix:from-row-list n-values N [n-values N [[0]]]

  reset-ticks
  setup-plot ; Initialize the plot
end

to setup-plot
  set-current-plot "Flocking Index" ; Make sure to set up a plot in the interface
  clear-plot ; Clear previous data
end

to move

  ;; First for each flocker update if they see other flockers
  ask flockers [
    let flock-i who
    let flock-i-turtle self

        ask other flockers [
            let flock-j who
            let flock-j-turtle self
            let dist distance flock-j-turtle
            matrix:set d flock-i flock-j dist

            ; Relative position vector (from flock-i to flock-j)
            let d_x ([xcor] of flock-j-turtle - [xcor] of flock-i-turtle)
            let d_y ([ycor] of flock-j-turtle - [ycor] of flock-i-turtle)

            ; Angle of the relative position vector
            let angle-to-flock-j atan dx dy  ; Angle in degrees

            let angle subtract-headings [heading] of flock-j-turtle [heading] of flock-i-turtle

            ifelse (dist <= r and abs angle-to-flock-j <= z / 2) [
                 matrix:set perceive flock-i flock-j 1
            ][
                 matrix:set perceive flock-i flock-j 0
             ]
            if (matrix:get FSR flock-i flock-j = 0 and dist <= P_d and matrix:get perceive flock-i flock-j = 1) [
               matrix:set FSR flock-i flock-j 1; set FSR to smooth mode
               matrix:set Di flock-i flock-j (dist + random-float (m - dist)) ;set ideal distance to random
            ]

        ]
  ]


  let best-flockers-moves[]
  let min-dissatisfactions-moves[]
  ;;Now for each flocker move based on the cell with the least possible dissatisfaction
  ask flockers[
    let flock-i who
    ;;Compute where to move based on dissatisfaction
    let min-dissatisfaction max-pxcor * max-pycor ; Start with a large value

    let empty-neighbors neighbors with [not any? turtles-here]

    ;;Calculate where it's best to move and move there
    if any? empty-neighbors [
      ;Compute for which cell the dissatisfaction would be minimum
      let best-moves empty-neighbors with-min  [
           dissatisfaction-for-patch self flock-i
      ]
      let best-move min-one-of best-moves [
        min-heading-for-patch self flock-i
      ]
       ;;Predict at which distance the flocker will be from agent j after moving to cell C if the agent j doesn't move
      ask other flockers[
        let flock-j who
       ;;Calculate predicted distance, if agents don't move
        let predicted-distance sqrt (([pxcor] of best-move - [x-coord] of self) ^ 2 + ([pycor] of best-move - [y-coord] of self ) ^ 2) ;; Calculate predicted distance
        matrix:set dp flock-i flock-j predicted-distance
      ]

      set best-flockers-moves lput best-move best-flockers-moves
      set min-dissatisfactions-moves lput (dissatisfaction-for-patch best-move flock-i) min-dissatisfactions-moves
      ;; move to the chosen cell

    ]

  ]


  ask flockers[
    let flock-i who
    let tried-moves []
    let best-move nobody
    let min-dissatisfaction max-pxcor * max-pycor
    let conflicts? true

    while[conflicts? and any? neighbors with [not member? self tried-moves and not any? turtles-here]][
      ;;show neighbors
      ;;show tried-moves
      let alternative-moves neighbors with [
        not any? turtles-here and not member? self tried-moves
      ]

      ;;print alternative-moves
      ifelse any? alternative-moves [
           let best-moves alternative-moves with-min  [
                dissatisfaction-for-patch self flock-i
           ]
           set best-move min-one-of best-moves [
                min-heading-for-patch self flock-i
           ]
      ][
        set best-move nobody
        set best-flockers-moves replace-item flock-i best-flockers-moves nobody ;; Update to the current patch
        set min-dissatisfactions-moves replace-item flock-i min-dissatisfactions-moves min-dissatisfaction
        stop
      ]

      set conflicts? false
      let conflicting-flockers []
      ask other flockers [
          let flock-j who

          let best-flock-move item flock-j best-flockers-moves
          if best-flock-move = best-move [
             set conflicts? true
             set conflicting-flockers lput flock-j conflicting-flockers
          ]
      ]
      if conflicts? [

        let dissatisfactions map [x -> dissatisfaction-for-patch best-move x] conflicting-flockers

        let min-conflict-dissatisfaction min dissatisfactions
           if min-dissatisfaction <= min-conflict-dissatisfaction [
          ;; Якщо наш dissatisfaction найменший або рівний, залишаємо клітинку собі
                set conflicts? false
           ]
      ]
      if best-move != nobody and not conflicts? [
          set best-flockers-moves replace-item flock-i best-flockers-moves best-move
          set min-dissatisfactions-moves replace-item flock-i min-dissatisfactions-moves min-dissatisfaction
      ]

      if best-move != nobody[
        set tried-moves lput best-move tried-moves
      ]

    ]

  ]

  ;;print best-flockers-moves

  ;; Move the flockers after predicting the distance and predicting the minimum dissatisfaction
  ask flockers[
       let flock-i who
        let best-move item flock-i best-flockers-moves
        if best-move != nobody [
        move-to best-move

        ;;Update coordinates and the heading
        let previous-x x-coord
        let previous-y y-coord
        set x-coord [pxcor] of best-move
        set y-coord [pycor] of best-move
        let d_x x-coord - previous-x
        let d_y y-coord - previous-y

        let new-heading-radians (atan d_x d_y) * pi / 180

        set heading-radians new-heading-radians

        set heading new-heading-radians * 180 / pi
      ]
  ]

  ;;Here a "tick" updates
  ;; We save all matrixes we calculated as matrixes of previous ticks
  let i 0
  let j 0
  repeat N - 1 [  ; Repeat N times (for rows)
    set j 0
    repeat N - 1 [  ; Repeat N times (for columns)
      ; Set the value in copied-matrix based on perceive_t

      matrix:set perceive_t-1 i j (matrix:get perceive i j)
      matrix:set d_t-1 i j (matrix:get d i j)
      matrix:set dp_t-1 i j (matrix:get dp i j)
      matrix:set Di_t-1 i j (matrix:get Di i j)

      set j j + 1

    ]
    set i i + 1
  ]

  ;; Update the distance matrix and the perceive matrix
  ask flockers[
    let flock-i who
    let flock-i-turtle self

    ask flockers with [who > flock-i] [
      let flock-j who
      let flock-j-turtle self
      let dist distance flock-j-turtle
      matrix:set d flock-i flock-j dist

      let angle subtract-headings [heading] of flock-j-turtle [heading] of flock-i-turtle

      ifelse (dist <= r and abs angle <= z / 2) [
          matrix:set perceive flock-i flock-j 1
       ][
          matrix:set perceive flock-i flock-j 0
        ]
    ]

  ]


  ;;Now for each flocker calculate how their expectations differ from reality, update the ideal distances
  ask flockers[
    ;; Update ideal distance
    let flock-i-turtle self
    ask other flockers[
      let flock-j-turtle self
      update-ideal-distance flock-i-turtle flock-j-turtle
    ]

  ]

  update-flocking-index ; Update the flocking index after moving
 ;
  tick
end

to-report dissatisfaction-for-patch [test-patch flock-i] ;Calculating dissatisfaction for each patch
  let dissatisfaction 0
  let perceived-agents-count 0

  ; Calculate dissatisfaction for each perceived flock member
  ask other flockers [
    ; Check if the agent is perceived
    if matrix:get perceive flock-i who = 1 [
      let dist-to-patch distance test-patch ; Distance to the test patch
      let ideal-dist matrix:get Di flock-i who ; Get ideal distance from matrix

      let u abs (dist-to-patch - ideal-dist)
      set dissatisfaction dissatisfaction + u
      set perceived-agents-count perceived-agents-count + 1
    ]
  ]

  ; Normalize dissatisfaction
  ifelse perceived-agents-count > 0 [
    report (dissatisfaction / perceived-agents-count) / m
  ][
    report max-pxcor * max-pycor ; Large value if no perceived agents
  ]
end

to-report min-heading-for-patch [test-patch flock-i]
    let agent-x [x-coord] of turtle flock-i
    let agent-y [y-coord] of turtle flock-i
    let agent-heading [heading] of turtle flock-i

    let d_x [pxcor] of test-patch - agent-x
    let d_y [pycor] of test-patch - agent-y

    ; Обчислення зміни напрямку
    let new-heading-radians atan d_y d_x
    let heading-change abs (new-heading-radians - agent-heading)

    ; Враховуємо, що поворот може бути менший при обчисленні в колі
    ifelse heading-change > 180 [
       report 360 - heading-change
    ][
       report heading-change
    ]
end


to update-flocking-index
  let total-flocking-index 0
  let num-pairs 0

  ; Define constants for the Z function
  let g 2.0  ; You can adjust this constant based on your model
  let r_z 0.5 ; The distance at which the function is steepest (adjust as needed)
  let m_z 1.0

  ; Loop over each flocking agent
  ask flockers [
    let flock-i who
    let flock-i-turtle self

    ; Compare with all other flockers (excluding itself)
    ask flockers with [who > flock-i] [
      let flock-j who
      let flock-j-turtle self

      ; Skip if flock-i is comparing with itself
      if flock-i != flock-j [

        ; Calculate the heading difference (Δαij)
        let delta-alpha abs ([heading-radians] of flock-i-turtle - [heading-radians] of flock-j-turtle)

        let delta-alpha-min min list delta-alpha (2 * pi - delta-alpha)  ; Use the minimum angular difference

        ; Calculate the H function for heading difference (H(Δαij))
        let H_fl delta-alpha-min / pi
        set H_fl 1 - H_fl

        ; Calculate the real distance between the two agents
        let distance-to-flocker matrix:get d flock-i flock-j

        ; Calculate the Z function for distance (Z(dij))
        let Z_fl 1 / (1 + exp (- g * (distance-to-flocker - r_z * m) / m))
        set Z_fl 1 - Z_fl
        if Z_fl < 0 or Z_fl > 1 [
          print(Z_fl)
        ]
         if H_fl < 0 or H_fl > 1 [
          print(H_fl)
        ]
        ; Aggregate the flocking index values for this pair
        set total-flocking-index total-flocking-index + (H_fl * Z_fl)
        set num-pairs num-pairs + 1
        ;;print H_fl
        ;;print Z_fl
      ]

    ]
  ]

  ;print total-flocking-index
  ;print N

  if num-pairs > 0 [
    ; Calculate the global flocking index as the mean of pairwise indices
    let global-flocking-index 2 * total-flocking-index / N / (N - 1)
    ;print global-flocking-index

    ; Update the global flocking index variable
    set flocking-index global-flocking-index

    ; Plot the global flocking index
    ;print (word "Global Flocking Index: " global-flocking-index)
    ;print (word "Flocking Index (to be plotted): " flocking-index)
    set-current-plot "Flocking Index"
    set-current-plot-pen "Global Flocking Index"
    plot flocking-index  ;; Plot at the current time (ticks) vs. the index
  ]
end

to update-kappa-index
  let total-kappa-index 1
  let average-flocking-index 1

  set total-kappa-index (flocking-index - average-flocking-index / ( 1 - average-flocking-index ) )
  set-current-plot "Kappa Index"
  set-current-plot-pen "kappa"
  plot flocking-index

end
to update-ideal-distance [flock-i flock-j]
  let i who
  let j who
  if matrix:get FSR i j = 0[
        stop
  ]

  ifelse  matrix:get FSR i j = 1[
  if(matrix:get perceive i j = true and matrix:get perceive_t-1 i j = true)[
    let c matrix:get dp i j - matrix:get d_t-1 i j
    let u abs (matrix:get d i j - matrix:get Di_t-1 i j) - abs (matrix:get dp i j - matrix:get Di_t-1 i j)
    let k 0
    if (c * u > 0) [ set k -1 ]
    if (c * u < 0) [ set k 1 ]
    if (c = 0 and u != 0) [ set k (ifelse-value (random 2 = 0) [1] [-1])]
    if (c = 0 and u = 0) [set k 0]

    matrix:set Di i j (1 + k * P_c) * matrix:get Di_t-1 i j
    matrix:set change i j k
    matrix:set cyclic i j k * matrix:get change_t-1 i j

    if (matrix:get change i j = 0 and matrix:get change_t-1 i j = 0) or (matrix:get cyclic i j < 0 and matrix:get cyclic_t-1 i j < 0)[
      let v matrix:get S_count-ier i j
      matrix:set S_count-ier i j v + 1

    ]

    if (matrix:get S_count-ier i j) >= (matrix:get Stag i j)[
    matrix:set FSR i j 2
    matrix:set Di i j matrix:get Di_t-1 i j + P_c * matrix:get Stag i j
    matrix:set Idle i j P_c * matrix:get Stag i j
    matrix:set Stag i j ( 1 + P_c) * matrix:get Stag i j

    matrix:set S_count-ier i j 0
    matrix:set E_count-ier i j 0
    ]

  ]

][
    let v matrix:get E_count-ier i j
    matrix:set E_count-ier i j v + 1

    if (matrix:get E_count-ier i j >= matrix:get Exil i j)
    [
       matrix:set FSR i j 1
       matrix:set Di i j (matrix:get Di_t-1 i j - matrix:get Idle i j - P_d)
       matrix:set Exil i j ( 1 - P_c) * matrix:get Exil i j
    ]

]

end


to go
  move
end

to simulate
  repeat 100 [go]
end
@#$#@#$#@
GRAPHICS-WINDOW
247
37
376
167
-1
-1
1.0
1
10
1
1
1
0
1
1
1
0
120
0
120
0
0
1
ticks
30.0

BUTTON
29
88
92
121
NIL
setup\n
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

PLOT
182
512
382
662
Flocking Index
NIL
NIL
0.0
1.0
0.0
1.0
true
false
"" ""
PENS
"Global Flocking Index" 1.0 0 -16777216 true "" "update-flocking-index"

BUTTON
31
131
94
164
NIL
move\n
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

CHOOSER
24
515
162
560
radius
radius
40 60 80
0

CHOOSER
25
573
163
618
circ-spector
circ-spector
225 270 315
0

SLIDER
28
427
200
460
P_s
P_s
5
25
9.0
1
1
NIL
HORIZONTAL

SLIDER
24
472
196
505
P_e
P_e
5
25
9.0
1
1
NIL
HORIZONTAL

SLIDER
27
383
199
416
P_d
P_d
0
100
60.0
1
1
NIL
HORIZONTAL

SLIDER
29
338
201
371
P_c
P_c
0
1
0.01
0.01
1
NIL
HORIZONTAL

CHOOSER
29
231
167
276
H
H
450 200 180 150 120
4

CHOOSER
28
285
166
330
W
W
600 260 230 210 200 180 150 120
7

BUTTON
30
175
107
208
NIL
simulate\n
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

@#$#@#$#@
## WHAT IS IT?

(a general understanding of what the model is trying to show or explain)

## HOW IT WORKS

(what rules the agents use to create the overall behavior of the model)

## HOW TO USE IT

(how to use the model, including a description of each of the items in the Interface tab)

## THINGS TO NOTICE

(suggested things for the user to notice while running the model)

## THINGS TO TRY

(suggested things for the user to try to do (move sliders, switches, etc.) with the model)

## EXTENDING THE MODEL

(suggested things to add or change in the Code tab to make the model more complicated, detailed, accurate, etc.)

## NETLOGO FEATURES

(interesting or unusual features of NetLogo that the model uses, particularly in the Code tab; or where workarounds were needed for missing features)

## RELATED MODELS

(models in the NetLogo Models Library and elsewhere which are of related interest)

## CREDITS AND REFERENCES

(a reference to the model's URL on the web if it has one, as well as any other necessary credits, citations, and links)
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

sheep
false
15
Circle -1 true true 203 65 88
Circle -1 true true 70 65 162
Circle -1 true true 150 105 120
Polygon -7500403 true false 218 120 240 165 255 165 278 120
Circle -7500403 true false 214 72 67
Rectangle -1 true true 164 223 179 298
Polygon -1 true true 45 285 30 285 30 240 15 195 45 210
Circle -1 true true 3 83 150
Rectangle -1 true true 65 221 80 296
Polygon -1 true true 195 285 210 285 210 240 240 210 195 210
Polygon -7500403 true false 276 85 285 105 302 99 294 83
Polygon -7500403 true false 219 85 210 105 193 99 201 83

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

wolf
false
0
Polygon -16777216 true false 253 133 245 131 245 133
Polygon -7500403 true true 2 194 13 197 30 191 38 193 38 205 20 226 20 257 27 265 38 266 40 260 31 253 31 230 60 206 68 198 75 209 66 228 65 243 82 261 84 268 100 267 103 261 77 239 79 231 100 207 98 196 119 201 143 202 160 195 166 210 172 213 173 238 167 251 160 248 154 265 169 264 178 247 186 240 198 260 200 271 217 271 219 262 207 258 195 230 192 198 210 184 227 164 242 144 259 145 284 151 277 141 293 140 299 134 297 127 273 119 270 105
Polygon -7500403 true true -1 195 14 180 36 166 40 153 53 140 82 131 134 133 159 126 188 115 227 108 236 102 238 98 268 86 269 92 281 87 269 103 269 113

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270
@#$#@#$#@
NetLogo 6.4.0
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180
@#$#@#$#@
0
@#$#@#$#@
