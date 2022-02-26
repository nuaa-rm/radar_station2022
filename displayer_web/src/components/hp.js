import React, {Component} from 'react';
import { Empty } from 'antd'
import { connect } from 'umi'

import HealthBar from "./healthBar";

@connect(({ robotStatus }) => ({
  robotStatus,
}))
class Hp extends Component {
  render() {
    console.log(this.props.robotStatus)
    let hpInfo = []
    const teams = ['red', 'blue']
    for (let i = 0; i < teams.length; i++) {
      const teamInfo = this.props.robotStatus[teams[i]]
      const robots = Object.keys(teamInfo)
      for (let j = 0; j < robots.length; j++) {
        hpInfo.push(
          <div style={{width: '100%'}} key={`${teams[i]}-${i}-info`}>
            <div>
              {robots[j]}
              <div style={{float: 'right'}}>
                {teamInfo[robots[j]].hp.toString() + "/" + teamInfo[robots[j]].hpLimit.toString()}
              </div>
            </div>
            <HealthBar width={this.props.width - 20} team={teams[i]} id={`${teams[i]}-${i}`}
                       hp={teamInfo[robots[j]].hp} hpLimit={teamInfo[robots[j]].hpLimit} />
          </div>
        )
      }
      hpInfo.push(<br key={teams[i] + '-br'} />)
    }
    return (
      <div style={{width: '100%', height: '100%', background: '#fff', padding: '10px'}}>
        <h2>HP Info</h2>
        {
          hpInfo.length > teams.length ?
            hpInfo :
            <Empty image={Empty.PRESENTED_IMAGE_SIMPLE} />
        }
      </div>
    );
  }
}

export default Hp;
