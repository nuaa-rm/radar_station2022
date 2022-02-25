import React, {Component} from 'react';
import { Progress, Empty } from 'antd'
import { connect } from 'umi'

@connect(({ robotStatus }) => ({
  robotStatus,
}))
class Hp extends Component {
  render() {
    console.log(this.props.robotStatus)
    let hpInfo = []
    const teams = ['red', 'blue']
    const color = ['red', '']
    for (let i = 0; i < teams.length; i++) {
      const teamInfo = this.props.robotStatus[teams[i]]
      const robots = Object.keys(teamInfo)
      for (let j = 0; j < robots.length; j++) {
        hpInfo.push(
          <div style={{width: '100%'}}>
            <div style={{width: 40}}>{robots[j]}</div>
            <div style={{width: '88%'}}>
              <Progress
                percent={teamInfo[robots[j]].hp / teamInfo[robots[j]].hpLimit * 100} strokeColor={color[i]} status="normal"
                format={()=>(teamInfo[robots[j]].hp.toString() + "/" + teamInfo[robots[j]].hpLimit.toString())}
                key={teams[i] + '-' + robots[j]}
              />
            </div>
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
