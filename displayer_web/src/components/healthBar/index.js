import React, {Component} from 'react';
import Progress from "./progress";

class Index extends Component {
  render() {
    const hpLimit = this.props.hpLimit
    const hp = this.props.hp
    const hpStep = 25
    const gridCount = hpLimit / hpStep
    let wholeGrid = Math.floor(gridCount)
    let totalWidth = wholeGrid * 40 + wholeGrid * 4
    const scale = (this.props.width - 10) / totalWidth
    const lastGrid = gridCount - wholeGrid
    const hpGrid = hp / hpStep
    const fullGrid = Math.floor(hpGrid)
    let lastHp = hpGrid - fullGrid
    if (lastGrid && fullGrid === wholeGrid) {
      lastHp *= lastGrid
    }
    let progresses = []
    wholeGrid = Math.ceil(wholeGrid)
    for (let i = 0; i < wholeGrid; i++) {
      let width = 0;
      let prog = 0;
      if (i === wholeGrid) {
        width = lastGrid
      }
      if (i < fullGrid) {
        prog = 1
      } else if (i === fullGrid) {
        prog = lastHp
      }
      progresses.push(
        <Progress width={width} progress={prog} team={this.props.team} key={this.props.id + i.toString()} />
      )
    }
    return (
      <div style={{width: this.props.width, height: 25, padding: 5, background: '#f0f2f5'}}>
        <div style={{width: totalWidth, height: '100%', transform: `scaleX(${scale})`, transformOrigin: '0% 50%'}}>
          {progresses}
        </div>
      </div>
    );
  }
}

export default Index;
