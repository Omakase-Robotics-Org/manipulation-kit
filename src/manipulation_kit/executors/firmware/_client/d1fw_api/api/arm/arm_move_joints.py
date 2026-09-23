from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.arm_move_joints_body import ArmMoveJointsBody
from ...models.arm_move_joints_response_200 import ArmMoveJointsResponse200
from ...models.arm_side import ArmSide
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs(
    side: ArmSide,
    *,
    body: ArmMoveJointsBody,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/arm/{side}/move_joints".format(
            side=quote(str(side), safe=""),
        ),
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ArmMoveJointsResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ArmMoveJointsResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ArmMoveJointsResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: ArmMoveJointsBody,
) -> Response[ArmMoveJointsResponse200 | ErrorEnvelope]:
    """Command one arm to a joint target

     Angles are degrees. With `wait: true` the call blocks until the arm reports arrival.

    The motion guard checks the straight joint-space path from the arm's current feedback pose to the
    target, with the other arm at its own feedback pose, sampled at 1 degree per joint: each sample is
    one dual-arm pose checked for body and chest keep-out, arm-arm and same-arm clearance at the
    configured margins, and the target is checked against the URDF joint limits. A path that fails is
    refused with `kind: refused` before anything is written, naming the sample, the moving joints and
    each pair under its margin with its distance; a path that starts inside a margin is accepted while
    no clearance gets smaller than at the start. The clearance guard cannot be switched off.

    The arm has to be in a mode that acts on a joint command (`position`, `pvt` or a torque mode). If it
    is not, the call is refused at once with `kind: refused` and nothing is written; an arm that is
    merely idle carries an `advisory` in the failure envelope naming the recover call that energises it.
    Without that refusal a waited call to an idle arm would sit out the full 30-second move timeout
    waiting for a convergence that servo-off joints cannot reach.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (ArmMoveJointsBody):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmMoveJointsResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        side=side,
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: ArmMoveJointsBody,
) -> ArmMoveJointsResponse200 | ErrorEnvelope | None:
    """Command one arm to a joint target

     Angles are degrees. With `wait: true` the call blocks until the arm reports arrival.

    The motion guard checks the straight joint-space path from the arm's current feedback pose to the
    target, with the other arm at its own feedback pose, sampled at 1 degree per joint: each sample is
    one dual-arm pose checked for body and chest keep-out, arm-arm and same-arm clearance at the
    configured margins, and the target is checked against the URDF joint limits. A path that fails is
    refused with `kind: refused` before anything is written, naming the sample, the moving joints and
    each pair under its margin with its distance; a path that starts inside a margin is accepted while
    no clearance gets smaller than at the start. The clearance guard cannot be switched off.

    The arm has to be in a mode that acts on a joint command (`position`, `pvt` or a torque mode). If it
    is not, the call is refused at once with `kind: refused` and nothing is written; an arm that is
    merely idle carries an `advisory` in the failure envelope naming the recover call that energises it.
    Without that refusal a waited call to an idle arm would sit out the full 30-second move timeout
    waiting for a convergence that servo-off joints cannot reach.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (ArmMoveJointsBody):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmMoveJointsResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        side=side,
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: ArmMoveJointsBody,
) -> Response[ArmMoveJointsResponse200 | ErrorEnvelope]:
    """Command one arm to a joint target

     Angles are degrees. With `wait: true` the call blocks until the arm reports arrival.

    The motion guard checks the straight joint-space path from the arm's current feedback pose to the
    target, with the other arm at its own feedback pose, sampled at 1 degree per joint: each sample is
    one dual-arm pose checked for body and chest keep-out, arm-arm and same-arm clearance at the
    configured margins, and the target is checked against the URDF joint limits. A path that fails is
    refused with `kind: refused` before anything is written, naming the sample, the moving joints and
    each pair under its margin with its distance; a path that starts inside a margin is accepted while
    no clearance gets smaller than at the start. The clearance guard cannot be switched off.

    The arm has to be in a mode that acts on a joint command (`position`, `pvt` or a torque mode). If it
    is not, the call is refused at once with `kind: refused` and nothing is written; an arm that is
    merely idle carries an `advisory` in the failure envelope naming the recover call that energises it.
    Without that refusal a waited call to an idle arm would sit out the full 30-second move timeout
    waiting for a convergence that servo-off joints cannot reach.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (ArmMoveJointsBody):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmMoveJointsResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        side=side,
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: ArmMoveJointsBody,
) -> ArmMoveJointsResponse200 | ErrorEnvelope | None:
    """Command one arm to a joint target

     Angles are degrees. With `wait: true` the call blocks until the arm reports arrival.

    The motion guard checks the straight joint-space path from the arm's current feedback pose to the
    target, with the other arm at its own feedback pose, sampled at 1 degree per joint: each sample is
    one dual-arm pose checked for body and chest keep-out, arm-arm and same-arm clearance at the
    configured margins, and the target is checked against the URDF joint limits. A path that fails is
    refused with `kind: refused` before anything is written, naming the sample, the moving joints and
    each pair under its margin with its distance; a path that starts inside a margin is accepted while
    no clearance gets smaller than at the start. The clearance guard cannot be switched off.

    The arm has to be in a mode that acts on a joint command (`position`, `pvt` or a torque mode). If it
    is not, the call is refused at once with `kind: refused` and nothing is written; an arm that is
    merely idle carries an `advisory` in the failure envelope naming the recover call that energises it.
    Without that refusal a waited call to an idle arm would sit out the full 30-second move timeout
    waiting for a convergence that servo-off joints cannot reach.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (ArmMoveJointsBody):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmMoveJointsResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            side=side,
            client=client,
            body=body,
        )
    ).parsed
